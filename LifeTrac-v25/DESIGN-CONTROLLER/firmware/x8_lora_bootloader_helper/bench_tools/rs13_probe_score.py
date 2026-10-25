#!/usr/bin/env python3
"""rs13_probe_score.py -- base->tractor command delivery from PROBE seqs (VECTOR_SCENE 8.6).

The range-edge procedure (RS13_RANGE_EDGE_LEG.md, Round A "command delivery",
and every DTS walk leg) scores a leg flown with -ReactiveFire 1 -ProbeEcho 0
on the tractor side: the distinct PROBE seqs in the tractor's
`LoRa cmd: PROBE seq=N ... (rx#M)` lines (tx_daemon.log) that also appear in
the base's `PROBE TX seq=N ... (tx#K)` lines (rx_daemon.log), over the base's
PROBE TX count. The base's own `probe:` line counts echoes, so with
-ProbeEcho 0 it reads 0 % and is ignored here.

    py -3 rs13_probe_score.py [--margin PP] [--void-unlocked]
                              [--scan-image auto|restarts|absorbing]
                              <archive_dir> [<archive_dir> ...]

Prints one block per archive (keep it as legs/leg<X>_probe_score.txt). Its
`tail:` line lists the base PROBE seqs sent after the last one the tractor
logged: they count as undelivered, but at the end of a leg they are often
end-of-log artefacts (the tractor's log was captured before they landed;
the harness's log-order fix removes most), so check them before quoting a
miss. Given legs of both codecs (params.txt cam_extra_env:
LIFETRAC_ENCODE_MODE=9 is VECTOR, =6 is mono_g4) on one profile, it pools
each codec per profile (params reg_profile) and prints the comparison: not
worse = VECTOR >= mono_g4, or a two-sided Fisher exact test on delivered /
undelivered with p >= 0.05. On DTS (reg_profile=2) that is the
VECTOR_SCENE 8.6 verdict; on FHSS (reg_profile=1) the same comparison
prints, labelled FHSS.

Fails closed. Exit 2 when a log is missing or holds no timestamped daemon
line (the tractor's is checked on a leg that fired probes; the harness
writes `docker logs` output, so a gone container or a failed adb call
leaves an empty or error-only file: that is no log, not a dead reverse
link), the tractor's log shows its image-tx-worker thread crashed, a fatal
HostLink open, or RXCONT arming disabled (-RxcontArm 0) (it was not
receiving), a PROBE line does not parse, the base's tx# or the
tractor's rx# counter skips (lines missing from a log, or a daemon restart
inside the leg), a base seq repeats, or the tractor logged a seq the base
never sent (the two logs are not from the same leg). Exit 1 when a leg
fired no probe (nothing to score; FHSS walk legs are image-only). Any failed
leg withholds the pooled verdict; so does (exit 1) a leg whose params.txt does
not show probe_echo=0 and a reg_profile of 1 (FHSS) or 2 (DTS), or whose codec
is neither VECTOR nor mono_g4, and a set of legs that mixes FHSS and DTS, since
the comparison pools VECTOR and mono_g4 legs of one profile flown with
-ProbeEcho 0 only (the per-leg figures still print). With --void-unlocked (Base
acquisition, below) a leg voided for a failed acquisition does not exit 1 for firing no
probe, but a set of legs it leaves nothing to pool exits 1. A real tractor log with no PROBE line
scores 0 delivered: that is a real result on a dead reverse link (RS-0.12
B1, 0/86), and the block says so loudly. Timestamps are not used to score:
the base and tractor clocks are not comparable.

Log window (2026-10-22). params.txt log_window says when the harness read the
daemon logs: after_stop = after both daemons stopped, so both logs end
together; absent = an older harness, which read the tractor's log and then
the base's while both daemons still ran, so a tail seq can be a read-window
artefact (2 of 10-19's 418/420 mono_g4 misses were). A leg that is not
after_stop (absent, or e.g. after_stop_unverified when the harness could not
confirm a stop) says so in its block, with log_window_detail when given, and
a set of legs whose log_window values differ (absent counts as one value) is
not pooled (exit 1): their tails are not counted alike.

Other opcodes (2026-10-22). On a -ReactiveFire 1 leg every base `command TX
opcode=` attempt should be the PROBE, 0x69. A `!!` line tallies the others
(0x60 REQ_KEYFRAME, 0x63 ENCODE_MODE, ...): control or profile traffic that
shared the command channel with the probes. On a 2d switch leg
leg2d_switch.sh sends 0x63 on purpose, so there it is expected. It withholds
nothing.

--margin PP (2026-10-22) adds a non-inferiority rule to the pooled
comparison: the pooled difference VECTOR minus mono_g4 in delivery, its
one-sided 95 % lower confidence bound (Newcombe's hybrid score interval,
method 10, built from Wilson score intervals), NOT WORSE (margin rule) when
that bound is above -PP and "not shown at this margin" otherwise, and roughly
how many probes per arm the bound would need to clear -PP if the observed
rates held. The Fisher lines print unchanged, so old outputs still compare,
but the Fisher rule passes whenever the sample is too small to tell (390/400
against 396/400 gives p = 0.176): a pass rule written before the legs should
name the margin. Pooling treats every probe as independent; leg-to-leg
variation is not modelled. Legs that share a log_window other than after_stop
still pool, but a `!!` line under the margin verdict says that it does not
meet a pass rule needing log_window=after_stop on every leg.

Base acquisition (2026-10-24, the FHSS round's leg 6). On an FHSS leg
(params reg_profile=1) each block gives the base's acquisition by
rs12_leg_report.py's rule (one rule, the archive's logs only) and then its
first image reception against the 120 s budget (4 scan attempts x 30 s), a
fact, not the verdict: a frame received is not a lock (the L072 delivers a
frame whose FHSS header it rejected). The rule reads the base log from its
profile-1 activation to its next REG_PROFILE activation (a profile switch
stops the scan). FAILED (`!! base ACQUISITION FAILED`) only on positive
evidence: a cold, or an early, final RX_SCAN_FAILED FAULT_URC, or an
instrument known to be on showing no frame within the budget (the
always-on stats rx_frames counter past it while the RX worker lives, or
per-fragment logging shown on by a frag_arrival line with none in it) that
nothing contradicts: a lock shown, or a daemon that logs FAULT_URC lines
with no RX_SCAN_FAILED one make that silence the host's, and the leg NOT
DETERMINED. ACQUIRED when a lock shows (the base decoded at least 25 % of
the tractor's fragments on air on a hop set of 10 or more channels, the
bench's 50 here, which only a grid follower does); NOT DETERMINED
otherwise. That is the rule of every L072 image before RS-12.21, whose scan
FAILED is absorbing. The RS-12.21 image (scan option (b); rs12_leg_report.py
tells it by the VER capability bitmap the base daemon logs, or by an
RX_SCAN_FAILED after a final one, and --scan-image overrides) restarts its
scan after a final and reports one every 30 s until a LOCK: there a final
is no FAILED on its own, a lock shown after it is a late ACQUIRED, and
silence counts over the whole window, not the budget; FAILED means the base
never locked onto the hop grid in the window (2026-10-24 review). The
block's line
reads `base acquisition: <verdict>; <first image reception>; evidence:
<what shows it>`. A failed base never follows the hop grid:
with -ReactiveFire 1 it fires a probe only after a frame it happened to
catch, from the channel it is parked on (on the RS-12.21 image, wherever
its scan stands), so the leg measures the
acquisition failure, not command delivery (leg 6: 4/5 pooled; a leg that
catches nothing fires no probe and withholds the pooled verdict). By
default nothing else changes: such a leg pools like any other, and one that
fired no probe still exits 1. --void-unlocked leaves every leg FAILED on
positive evidence out of the pool (and out of the log-window and profile
checks) and lists them, so a leg that fired no probe for that reason no
longer withholds the verdict; a leg NOT DETERMINED or n/a stays in. When it
leaves no leg to pool in a set of two or more, it prints `pooled: no leg
left to pool after --void-unlocked` and exits 1. Use it only when a pass
rule written before leg 1 says so: dropping a leg after seeing its numbers
is a choice made after the data.
"""
from __future__ import annotations

import argparse
import math
import os
import re
import sys
from fractions import Fraction
from pathlib import Path
from statistics import NormalDist

DC = Path(__file__).resolve().parents[3]       # bench_tools -> helper -> firmware -> DC
if str(DC / "tools") not in sys.path:
    sys.path.insert(0, str(DC / "tools"))
import rs12_leg_report as rep  # noqa: E402  (the acquisition rule, 2026-10-24)

_BASE_MARK = re.compile(r"PROBE TX\b")
_BASE_TX = re.compile(r"PROBE TX seq=(\d+) phase_ms=(-?\d+) size_b=(\d+) \(tx#(\d+)\)")
_TR_MARK = re.compile(r"LoRa cmd: PROBE\b")
_TR_RX = re.compile(r"LoRa cmd: PROBE seq=(\d+) phase_ms=(-?\d+) \(rx#(\d+)\)")
# image_rx_daemon._send_command_frame logs every attempt; PROBE is opcode 0x69.
_ON_AIR = re.compile(r"command TX opcode=0x69 copy=\d+/\d+ (OK \(on air\)|FAILED)")
# The same attempt line for any opcode (2026-10-22: the other-opcode tally).
_CMD_TX = re.compile(r"command TX opcode=0x([0-9a-fA-F]{2}) copy=\d+/\d+ (OK \(on air\)|FAILED)")
PROBE_OPCODE = 0x69
# lora_proto.py CMD_OP_* (base -> tractor), for the other-opcode tally.
OPCODE_NAME = {0x60: "REQ_KEYFRAME", 0x63: "ENCODE_MODE", 0x65: "RADIO_PROFILE",
               0x67: "RADIO_PROFILE_CONF", 0x6B: "CTRL_DITTO", 0x6C: "TILE_STALE"}
_MODE = re.compile(r"LIFETRAC_ENCODE_MODE=(\d+)")
# A daemon log line (Python logging's asctime): proof the file is a daemon log.
_TS_LINE = re.compile(r"^\ufeff?\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3} ", re.M)
CODEC_BY_MODE = {"9": "vector", "6": "mono_g4"}
# params reg_profile values whose legs pool (host_cfg_keys.h REG_PROFILE_*)
PROFILE_LABEL = {"1": "FHSS", "2": "DTS"}
# params log_window: the harness read both daemon logs after both had stopped.
LOG_WINDOW_AFTER_STOP = "after_stop"
ALPHA = 0.05
# The margin rule's one-sided 95 % bound: z = 1.6449, i.e. the lower end of
# the two-sided 90 % interval.
Z_ONE_SIDED_95 = NormalDist().inv_cdf(0.95)
TAIL_SHOWN = 20


class ScoreError(Exception):
    """The logs cannot be scored as they are (exit 2)."""


def _read(path: str) -> str:
    if not os.path.isfile(path):
        raise ScoreError(f"missing {path}")
    with open(path, encoding="utf-8-sig", errors="replace") as fh:
        return fh.read()


def parse_params(text: str) -> dict[str, str]:
    out = {}
    for line in text.splitlines():
        key, sep, val = line.lstrip("\ufeff").partition("=")
        if sep:
            out[key.strip()] = val.strip()
    return out


def leg_codec(params: dict[str, str]) -> str:
    m = _MODE.search(params.get("cam_extra_env", ""))
    if m:
        return CODEC_BY_MODE.get(m.group(1), f"encode mode {m.group(1)}")
    if params.get("tx_feed") == "local":
        return "synthetic (tx_feed=local)"
    return "unknown (no LIFETRAC_ENCODE_MODE in cam_extra_env)"


def _collect(text: str, mark: re.Pattern, pat: re.Pattern, what: str) -> list[tuple]:
    out = []
    for n, line in enumerate(text.splitlines(), 1):
        if not mark.search(line):
            continue
        m = pat.search(line)
        if not m:
            raise ScoreError(f"{what} line {n} does not parse: {line.strip()[:160]!r}")
        out.append(tuple(int(g) for g in m.groups()))
    return out


def _check_counter(counts: list[int], what: str) -> None:
    """tx# / rx# count from 1 at daemon start and step by one per line."""
    for i, c in enumerate(counts, 1):
        if c != i:
            raise ScoreError(
                f"{what} counter reads #{c} where line {i} expects #{i}: lines are "
                f"missing from the log, or the daemon restarted inside the leg")


def score_leg(rx_text: str, tx_text: str) -> dict:
    """Score one leg from the base (rx_daemon) and tractor (tx_daemon) logs.

    Returns ``sent`` (base PROBE TX lines), ``delivered`` (distinct tractor
    seqs that the base sent), the tractor line count and duplicates, the
    base's on-air / failed attempts for opcode 0x69 (None where the base log
    predates that line), and ``other_ops``: {opcode: (attempts, on air)} for
    every other opcode the base attempted. Raises ScoreError on anything that
    would make the figure wrong rather than low, including a log that is not
    a daemon log (the tractor's is checked only when the base fired a probe)."""
    if not _TS_LINE.search(rx_text):
        raise ScoreError("rx_daemon.log holds no timestamped daemon line: the base log is "
                         "missing or empty (a failed `docker logs rx_smoke`)")
    base = _collect(rx_text, _BASE_MARK, _BASE_TX, "base PROBE TX")
    if base and not _TS_LINE.search(tx_text):
        raise ScoreError(
            f"tx_daemon.log holds no timestamped daemon line "
            f"({sum(1 for line in tx_text.splitlines() if line.strip())} non-blank lines): "
            f"the tractor log is missing or empty (a failed `docker logs tx_smoke`), not a "
            f"dead reverse link")
    if base:
        # The tractor could not receive while its worker was dead or its radio never
        # opened: probes sent then would count as link loss.
        for mark, why in (("Exception in thread image-tx-worker", "its image-tx-worker thread "
                           "crashed"),
                          ("fatal: cannot open L072 HostLink", "it never opened its L072 HostLink"),
                          ("RXCONT arming DISABLED", "it ran with -RxcontArm 0, its command "
                           "downlink deaf by design")):
            if mark in tx_text:
                raise ScoreError(f"tx_daemon.log: {why} (`{mark}`): the tractor was not receiving, "
                                 f"so a low delivery would measure the daemon, not the link")
    trac = _collect(tx_text, _TR_MARK, _TR_RX, "tractor LoRa cmd: PROBE")
    _check_counter([b[3] for b in base], "base PROBE TX tx#")
    _check_counter([t[2] for t in trac], "tractor PROBE rx#")
    base_seqs = [b[0] for b in base]
    if len(set(base_seqs)) != len(base_seqs):
        dup = sorted({s for s in base_seqs if base_seqs.count(s) > 1})[:10]
        raise ScoreError(f"base PROBE seq(s) repeat: {dup} -- seqs cannot identify probes")
    tr_seqs = [t[0] for t in trac]
    stray = sorted(set(tr_seqs) - set(base_seqs))
    if stray:
        raise ScoreError(
            f"the tractor logged {len(stray)} PROBE seq(s) the base never sent in this "
            f"log ({stray[:10]}): the two logs are not from the same leg")
    on_air = _ON_AIR.findall(rx_text)
    ok = sum(1 for o in on_air if o.startswith("OK"))
    # Every other opcode the base attempted (2026-10-22): on a -ReactiveFire 1 leg
    # the base should send PROBEs only, so these are control or profile traffic
    # that shared the command channel with the probes.
    other_ops: dict[int, tuple[int, int]] = {}
    for op_hex, outcome in _CMD_TX.findall(rx_text):
        op = int(op_hex, 16)
        if op != PROBE_OPCODE:
            n, n_ok = other_ops.get(op, (0, 0))
            other_ops[op] = (n + 1, n_ok + outcome.startswith("OK"))
    # The tail: base seqs sent after the last one the tractor logged (in the base's
    # send order). Undelivered by construction; at the end of a leg, often probes
    # that landed after the tractor's log was captured. Informational only.
    tail = tail_after = None
    if tr_seqs:
        pos = {s: i for i, s in enumerate(base_seqs)}
        last = max(pos[s] for s in set(tr_seqs))
        tail, tail_after = base_seqs[last + 1:], base_seqs[last]
    return {
        "sent": len(base), "delivered": len(set(tr_seqs)),
        "tractor_lines": len(trac), "duplicates": len(tr_seqs) - len(set(tr_seqs)),
        "seq_first": base_seqs[0] if base_seqs else None,
        "seq_last": base_seqs[-1] if base_seqs else None,
        "on_air": ok if on_air else None,
        "failed": len(on_air) - ok if on_air else None,
        "tx_daemon_lines": sum(1 for line in tx_text.splitlines() if line.strip()),
        "tail": tail, "tail_after": tail_after,
        "other_ops": dict(sorted(other_ops.items())),
    }


def fisher_two_sided(a: int, b: int, c: int, d: int) -> float:
    """Two-sided Fisher exact p for [[a, b], [c, d]]: the sum of the
    hypergeometric probabilities of every table with the same margins that is
    no more likely than the observed one."""
    n1, n2, k = a + b, c + d, a + c
    n = n1 + n2
    if n == 0:
        return 1.0
    denom = math.comb(n, k)

    def prob(x: int) -> float:
        return math.comb(n1, x) * math.comb(n2, k - x) / denom

    p_obs = prob(a)
    p = sum(q for q in (prob(x) for x in range(max(0, k - n2), min(k, n1) + 1))
            if q <= p_obs * (1 + 1e-9))
    return min(1.0, p)


def wilson_interval(x: float, n: float, z: float) -> tuple[float, float]:
    """Wilson score interval (no continuity correction) for x successes in n:
    the p0 where |x/n - p0| = z sqrt(p0 (1 - p0) / n), in closed form. x may
    be fractional (probes_per_arm_to_clear scales the counts)."""
    den = n + z * z
    centre = (x + z * z / 2) / den
    half = z / den * math.sqrt(max(0.0, x * (n - x) / n) + z * z / 4)
    return max(0.0, centre - half), min(1.0, centre + half)


def newcombe_interval(x1: float, n1: float, x2: float, n2: float,
                      z: float) -> tuple[float, float]:
    """Newcombe's hybrid score interval for p1 - p2 (method 10 of Newcombe
    1998, Stat Med 17:873-890): each proportion's Wilson limits l, u, then
    p1 - p2 - sqrt((p1 - l1)^2 + (u2 - p2)^2) to
    p1 - p2 + sqrt((u1 - p1)^2 + (p2 - l2)^2).

    Checked (2026-10-22) against the paper's worked examples, Table II method
    10 at 95 % (z = 1.96): 56/70 - 48/80 gives 0.0524 to 0.3339, 9/10 - 3/10
    0.1705 to 0.8090, 10/10 - 0/20 0.6791 to 1.0000 (the SIL tests pin them),
    and the Wilson limits against an independent bisection of the score-test
    equation (also in the tests)."""
    p1, p2 = x1 / n1, x2 / n2
    l1, u1 = wilson_interval(x1, n1, z)
    l2, u2 = wilson_interval(x2, n2, z)
    d = p1 - p2
    return (d - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2),
            d + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2))


def probes_per_arm_to_clear(p1: float, p2: float, margin: float,
                            z: float = Z_ONE_SIDED_95, limit: int = 10 ** 9) -> int | None:
    """Smallest n such that the Newcombe lower bound of p1 - p2, with n probes
    per arm at exactly these rates, is above -margin (fractions, not pp).
    None when p1 - p2 <= -margin (no n will do) or n would exceed ``limit``;
    a shortfall of exactly the margin can land on either side of that float
    test and then returns None by the limit, so a caller holding the counts
    decides that case exactly (margin_rule does).

    Approximate: it assumes the rates hold exactly, so at that n the bound
    clears about half the time; it is not a power calculation. It searches
    the score bound itself rather than the normal approximation
    n = z^2 (p1 q1 + p2 q2) / (p1 - p2 + margin)^2, which has no variance
    at 100 % (10-19: VECTOR 433/433) and then asks for far too few probes.
    The bound rises with n at fixed rates (both Wilson half-widths shrink),
    so a doubling search and a bisection find it."""
    if p1 - p2 <= -margin:
        return None

    def clears(n: int) -> bool:
        return newcombe_interval(p1 * n, n, p2 * n, n, z)[0] > -margin

    hi = 1
    while not clears(hi):
        hi *= 2
        if hi > limit:
            return None
    lo = hi // 2
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if clears(mid):
            hi = mid
        else:
            lo = mid
    return hi


def _pct(x: int, y: int) -> str:
    return f"{100.0 * x / y:.1f} %" if y else "n/a"


def leg_acquisition(params: dict[str, str], rx_text: str, tx_text: str,
                    image: str = "auto") -> dict | None:
    """rs12_leg_report.acquisition on an FHSS leg (params reg_profile=1), else None;
    ``image``: --scan-image (rs12_leg_report.scan_image)."""
    if params.get("reg_profile") != "1":
        return None
    return rep.acquisition(rx_text, tx_text, image=image)


def acquisition_lines(acq: dict | None) -> list[str]:
    """The per-leg base acquisition line (2026-10-24): rs12_leg_report's verdict,
    then the first image reception (a fact, not the verdict), then the verdict's
    evidence; none off FHSS."""
    if acq is None:
        return []
    if not acq["applies"]:
        return [f"  base acquisition: n/a ({acq['why']})"]
    seen = rep.reception_phrase(acq)
    if acq["status"] != rep.FAILED:
        return [f"  base acquisition: {acq['status']}; {seen}; evidence: {acq['basis']}"]
    if acq.get("restarts"):
        # 2026-10-24 review: the RS-12.21 image scans on after a final
        return [f"  !! base ACQUISITION FAILED; {seen}; evidence: {acq['basis']}. Its scan "
                f"scanned on (the RS-12.21 image restarts it after each final RX_SCAN_FAILED) "
                f"but never locked onto the hop grid, so any probe it fired went out off the "
                f"grid, after a frame it happened to catch; this leg measures that, not "
                f"command delivery (--void-unlocked leaves it out of the pool, only under a "
                f"pass rule written before leg 1)"]
    return [f"  !! base ACQUISITION FAILED; {seen}; evidence: {acq['basis']}. Its scan ran out "
            f"into FAILED and never followed the hop grid, so any probe it fired went out from "
            f"the channel it was parked on, after a frame it happened to catch; this leg "
            f"measures that, not command delivery (--void-unlocked leaves it out of the pool, "
            f"only under a pass rule written before leg 1)"]


def format_leg(name: str, params: dict[str, str], res: dict,
               acq: dict | None = None) -> list[str]:
    out = [f"{name}: codec {leg_codec(params)}, reg_profile={params.get('reg_profile', '?')}, "
           f"reactive_fire={params.get('reactive_fire', '?')}, "
           f"probe_echo={params.get('probe_echo', '?')}, "
           f"duration_s={params.get('duration_s', '?')}"]
    air = ("n/a (base log predates the per-attempt command TX line)" if res["on_air"] is None
           else f"on air {res['on_air']}, failed {res['failed']}")
    out.append(f"  base PROBE TX: {res['sent']} (seq {res['seq_first']}..{res['seq_last']}; {air})")
    out.append(f"  tractor LoRa cmd: PROBE: {res['tractor_lines']} line(s), "
               f"{res['delivered']} distinct seq(s), duplicates {res['duplicates']}")
    out.append(f"  delivered {res['delivered']}/{res['sent']} = "
               f"{_pct(res['delivered'], res['sent'])}  (tractor seqs that the base sent, over "
               f"base PROBE TX)")
    if res["on_air"] is not None and res["on_air"] + res["failed"] != res["sent"]:
        out.append(f"  !! {res['on_air'] + res['failed']} opcode-0x69 attempts against "
                   f"{res['sent']} PROBE TX lines: check the base log")
    if res["failed"]:
        out.append(f"  note: {res['failed']} PROBE(s) never went on air (TX FAILED at the "
                   f"base); they count as undelivered, as the procedure's denominator says")
    tail = res.get("tail")
    if tail:
        shown = ", ".join(str(s) for s in tail[:TAIL_SHOWN]) + (", ..." if len(tail) > TAIL_SHOWN
                                                                 else "")
        out.append(f"  tail: {len(tail)} base PROBE seq(s) sent after seq {res['tail_after']}, the "
                   f"tractor's last logged: [{shown}] (counted undelivered; at a leg's end often "
                   f"an end-of-log artefact: check before quoting a miss)")
    elif tail is not None:
        out.append(f"  tail: none (the tractor logged the base's last PROBE, seq "
                   f"{res['tail_after']})")
    if res["tractor_lines"] == 0:
        out.append(f"  !! the tractor log has no PROBE line at all ({res['tx_daemon_lines']} "
                   f"non-blank lines): 0 delivered. Real on a dead reverse link; check the "
                   f"tx_daemon.log is this leg's before quoting it")
    if params.get("reg_profile") not in (None, *PROFILE_LABEL):
        out.append(f"  note: reg_profile={params['reg_profile']} is neither FHSS (1) nor DTS (2): "
                   f"this leg pools with no other")
    if params.get("probe_echo") not in (None, "0"):
        out.append("  !! probe_echo is on: the tractor's echo deafens the window the next "
                   "command needs (RS-0.12: 56 % with echo, 91 % without), so this score "
                   "does not compare with -ProbeEcho 0 legs")
    others = res.get("other_ops") or {}
    if params.get("reactive_fire") == "1" and others:
        tally = ", ".join(f"0x{op:02x}{' ' + OPCODE_NAME[op] if op in OPCODE_NAME else ''} "
                          f"{n} ({n_ok} on air)" for op, (n, n_ok) in others.items())
        line = (f"  !! base command TX attempts other than PROBE (0x69) on this -ReactiveFire 1 "
                f"leg: {tally}: control or profile traffic shared the command channel with "
                f"the probes")
        if 0x63 in others:
            line += (" (0x63 is expected only on a 2d switch leg, where leg2d_switch.sh sends it "
                     "on purpose; on any other leg it is unplanned traffic)")
        out.append(line)
    window = params.get("log_window")
    if not window:
        out.append("  log window: params.txt has no log_window, so an older harness read the "
                   "tractor's log, then the base's, while both daemons still ran: a tail seq can "
                   "be a read-window artefact. Pools only with legs read the same way")
    elif window != LOG_WINDOW_AFTER_STOP:
        # e.g. after_stop_unverified: a docker stop at the leg's end was not confirmed,
        # and the harness gives its reason in log_window_detail.
        detail = params.get("log_window_detail")
        out.append(f"  log window: log_window={window}, not {LOG_WINDOW_AFTER_STOP}"
                   + (f" (log_window_detail: {detail})" if detail else "")
                   + ": pools only with legs of the same log_window")
    return out + acquisition_lines(acq)


def pooled_verdict(vec: tuple[int, int], mono: tuple[int, int], profile: str = "2",
                   margin_pp: float | None = None) -> list[str]:
    """vec / mono: (delivered, sent) pooled per codec, all legs on ``profile``
    (params reg_profile). DTS ("2") prints the VECTOR_SCENE 8.6 verdict; any
    other profile the same comparison, labelled with it. With ``margin_pp``
    the margin rule's lines follow the Fisher lines, which do not change."""
    (dv, sv), (dm, sm) = vec, mono
    p = fisher_two_sided(dv, sv - dv, dm, sm - dm)
    if profile == "2":
        pool, head = "pooled ", "command delivery (VECTOR_SCENE 8.6)"
    else:
        lab = PROFILE_LABEL.get(profile, f"reg_profile={profile}")
        pool = f"pooled {lab} "
        head = (f"command delivery, {lab} legs (the VECTOR_SCENE 8.6 comparison; the procedure "
                f"scores 8.6 on DTS)")
    out = [f"{pool}VECTOR {dv}/{sv} = {_pct(dv, sv)}; {pool}mono_g4 {dm}/{sm} = {_pct(dm, sm)}",
           f"two-sided Fisher exact on delivered / undelivered: p = {p:.3g}"]
    if dv * sm >= dm * sv:
        out.append(f"{head}: NOT WORSE (VECTOR >= mono_g4)")
    elif p >= ALPHA:
        out.append(f"{head}: NOT WORSE (VECTOR below mono_g4, Fisher p = {p:.3g} >= {ALPHA})")
    else:
        out.append(f"{head}: WORSE (VECTOR below mono_g4, Fisher p = {p:.3g} < {ALPHA})")
    if margin_pp is not None:
        out += margin_rule(vec, mono, margin_pp, head)
    return out


def margin_rule(vec: tuple[int, int], mono: tuple[int, int], margin_pp: float,
                head: str) -> list[str]:
    """The --margin lines (2026-10-22): VECTOR minus mono_g4 in delivery, its
    one-sided 95 % Newcombe lower bound, the verdict against -margin_pp, and
    the approximate probes per arm that bound needs at the observed rates."""
    (dv, sv), (dm, sm) = vec, mono
    p1, p2, m = dv / sv, dm / sm, margin_pp / 100.0
    # Whether no probe count can clear the margin, decided exactly on the counts
    # (2026-10-22 review): a shortfall of exactly the margin, e.g. 184/200 against
    # 188/200 at 2 pp, comes out a hair above -m in floats.
    hopeless = Fraction(dv, sv) - Fraction(dm, sm) <= -Fraction(repr(margin_pp)) / 100
    lo = newcombe_interval(dv, sv, dm, sm, Z_ONE_SIDED_95)[0]
    out = [f"margin rule, {margin_pp:g} pp: VECTOR minus mono_g4 delivery "
           f"{100 * (p1 - p2):+.2f} pp, one-sided 95 % lower bound {100 * lo:+.2f} pp "
           f"(Newcombe hybrid score interval, method 10)"]
    if lo > -m:
        out.append(f"{head}, margin {margin_pp:g} pp: NOT WORSE (margin rule): the lower bound "
                   f"{100 * lo:+.2f} pp is above -{margin_pp:g} pp")
    else:
        out.append(f"{head}, margin {margin_pp:g} pp: not shown at this margin: the lower bound "
                   f"{100 * lo:+.2f} pp is not above -{margin_pp:g} pp")
    n = None if hopeless else probes_per_arm_to_clear(p1, p2, m)
    if n is None:
        why = ("no number would do: VECTOR's observed shortfall, "
               f"{100 * (p2 - p1):.2f} pp, is not smaller than the margin" if hopeless
               else "more than 10^9")
        out.append(f"  probes per arm for the bound to clear -{margin_pp:g} pp at these rates: "
                   f"{why}")
    else:
        out.append(f"  probes per arm for the bound to clear -{margin_pp:g} pp if these rates "
                   f"held: ~{n} (approximate: the same bound at these rates with n per arm, so "
                   f"about an even chance of clearing at n, not a power calculation; this pool "
                   f"has {sv} VECTOR / {sm} mono_g4)")
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("archive", nargs="+",
                    help="harness archive dir(s): radio_monitor_<stamp>_<sha>")
    ap.add_argument("--margin", type=float, metavar="PP",
                    help="non-inferiority margin in percentage points: adds the margin rule "
                         "(one-sided 95 %% Newcombe lower bound of VECTOR minus mono_g4 "
                         "delivery against -PP) to the pooled comparison")
    ap.add_argument("--void-unlocked", action="store_true",
                    help="leave the FHSS legs whose base acquisition FAILED on positive "
                         "evidence (rs12_leg_report.py's rule; not one NOT DETERMINED) out of "
                         "the pool and list them; only under a pass rule written before leg 1 "
                         "that says so")
    ap.add_argument("--scan-image", choices=rep.SCAN_IMAGES, default="auto",
                    help="how the base's L072 image treats a final RX_SCAN_FAILED, for the "
                         "acquisition rule (rs12_leg_report.py --scan-image): 'restarts' "
                         "(the RS-12.21 image), 'absorbing' (every image before it), 'auto' "
                         "(default: the logged VER capability bitmap, else the faults)")
    a = ap.parse_args(argv)
    if a.margin is not None and not 0 < a.margin < 100:
        ap.error(f"--margin {a.margin:g}: give a margin between 0 and 100 percentage points")
    rc = 0
    pools: dict[tuple[str, str], list[int]] = {}     # (reg_profile, codec) -> [delivered, sent]
    unlike: list[str] = []                   # legs not shown to be FHSS / DTS with -ProbeEcho 0
    other: list[str] = []                    # legs that are neither VECTOR nor mono_g4
    scored: list[str] = []                   # every scored leg, with its profile and echo
    windows: dict[str, list[str]] = {}       # params log_window ("" = absent) -> legs
    voided: list[str] = []                   # --void-unlocked: legs whose acquisition failed
    for arch in a.archive:
        name = os.path.basename(os.path.normpath(arch))
        try:
            params = parse_params(_read(os.path.join(arch, "params.txt")))
            rx_text = _read(os.path.join(arch, "rx_daemon.log"))
            tx_text = _read(os.path.join(arch, "tx_daemon.log"))
            res = score_leg(rx_text, tx_text)
        except ScoreError as exc:
            print(f"{name}: CANNOT SCORE: {exc}")
            rc = 2
            continue
        # 2026-10-24: the base's FHSS acquisition (rs12_leg_report's rule); only a
        # leg FAILED on positive evidence voids, never one NOT DETERMINED
        acq = leg_acquisition(params, rx_text, tx_text, a.scan_image)
        void = a.void_unlocked and bool(acq and acq.get("status") == rep.FAILED)
        if res["sent"] == 0:
            print(f"{name}: no PROBE TX line in rx_daemon.log (reactive_fire="
                  f"{params.get('reactive_fire', '?')}): this leg fired no probe, so it has "
                  f"no command-delivery score (DTS legs need -ReactiveFire 1)")
            for line in acquisition_lines(acq):
                print(line)
            if void:
                voided.append(name)
            else:
                rc = max(rc, 1)
            continue
        for line in format_leg(name, params, res, acq):
            print(line)
        if void:
            voided.append(name)
            continue
        codec = leg_codec(params)
        prof = params.get("reg_profile", "?")
        pool = pools.setdefault((prof, codec), [0, 0])
        if codec not in ("vector", "mono_g4"):
            other.append(f"{name} ({codec})")
        pool[0] += res["delivered"]
        pool[1] += res["sent"]
        leg = f"{name} (reg_profile={prof}, probe_echo={params.get('probe_echo', '?')})"
        scored.append(leg)
        if prof not in PROFILE_LABEL or params.get("probe_echo") != "0":
            unlike.append(leg)
        windows.setdefault(params.get("log_window", ""), []).append(name)
    if voided:
        # 2026-10-24: listed before any pooled line, so the reader sees what the
        # verdict leaves out
        print(f"voided by --void-unlocked (the base's FHSS acquisition FAILED on positive "
              f"evidence, each block's !! line says which), left out of the pool: "
              f"{', '.join(voided)}. Valid only under a pass rule written before leg 1 that "
              f"says so")
    if len(a.archive) > 1:
        codecs = {c for _, c in pools}
        profiles = sorted({p for p, _ in pools})
        both = "vector" in codecs and "mono_g4" in codecs
        if rc:
            print("pooled verdict withheld: a leg above could not be scored")
        elif not pools:
            print("pooled: no leg left to pool after --void-unlocked")
            rc = 1
        elif other and both:
            print("pooled verdict withheld: the 8.6 comparison pools VECTOR and mono_g4 legs "
                  "only (params cam_extra_env LIFETRAC_ENCODE_MODE=9 / 6), and these legs are "
                  "neither: " + "; ".join(other) + ". The per-leg figures above stand")
            rc = 1
        elif unlike and both:
            print("pooled verdict withheld: the comparison pools legs flown with -ProbeEcho 0 "
                  "on FHSS or DTS only (params probe_echo=0, reg_profile=1 or 2), and these legs "
                  "are not shown to be: " + "; ".join(unlike) + ". The per-leg figures above stand")
            rc = 1
        elif len(profiles) > 1 and both:
            print(f"pooled verdict withheld: the legs mix reg_profile {' and '.join(profiles)} ("
                  + "; ".join(scored) + "); the comparison pools the legs of one profile. The "
                  "per-leg figures above stand")
            rc = 1
        elif len(windows) > 1:
            # 2026-10-22: a log read while its daemon ran ends at another moment than
            # one read after the stop, so the tails do not count alike. The pools are
            # not split by window, so even one codec's figures are withheld.
            groups = "; ".join(
                (f"log_window={w}" if w else "no log_window (read while running)")
                + ": " + ", ".join(legs) for w, legs in windows.items())
            print(f"pooled {'verdict' if both else 'figures'} withheld: the legs mix log windows ("
                  + groups + "); a log read while its daemon ran ends at another moment than "
                  "one read after the stop, so their tails do not count alike: pool the legs of "
                  "one log window. The per-leg figures above stand")
            rc = 1
        elif both:
            prof = profiles[0]
            for line in pooled_verdict(tuple(pools[(prof, "vector")]),
                                       tuple(pools[(prof, "mono_g4")]), prof, a.margin):
                print(line)
            window = next(iter(windows))
            if a.margin is not None and window != LOG_WINDOW_AFTER_STOP:
                # 2026-10-22 review: one shared window pools, but a pass rule needs
                # after_stop on every leg, so the margin verdict says when it is not.
                print("!! the margin verdict above pools legs with "
                      + (f"log_window={window}" if window else
                         "no log_window (read while running)")
                      + f", not {LOG_WINDOW_AFTER_STOP}: their tail seqs can be read-window "
                      "artefacts, so it does not meet a pass rule that needs log_window="
                      f"{LOG_WINDOW_AFTER_STOP} on every leg (re-fly them, or drop them under "
                      "a rule written before leg 1)")
        else:
            print("pooled: " + "; ".join(
                f"{PROFILE_LABEL.get(p, f'reg_profile={p}')} {c} {d}/{s} = {_pct(d, s)}"
                for (p, c), (d, s) in pools.items())
                + " -- no VECTOR_SCENE 8.6 comparison (it needs VECTOR and mono_g4 legs)")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
