#!/usr/bin/env python3
"""rs13_probe_score.py -- base->tractor command delivery from PROBE seqs (VECTOR_SCENE 8.6).

The range-edge procedure (RS13_RANGE_EDGE_LEG.md, Round A "command delivery",
and every DTS walk leg) scores a leg flown with -ReactiveFire 1 -ProbeEcho 0
on the tractor side: the distinct PROBE seqs in the tractor's
`LoRa cmd: PROBE seq=N ... (rx#M)` lines (tx_daemon.log) that also appear in
the base's `PROBE TX seq=N ... (tx#K)` lines (rx_daemon.log), over the base's
PROBE TX count. The base's own `probe:` line counts echoes, so with
-ProbeEcho 0 it reads 0 % and is ignored here.

    py -3 rs13_probe_score.py <archive_dir> [<archive_dir> ...]

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
-ProbeEcho 0 only (the per-leg figures still print). A real tractor log with no PROBE line
scores 0 delivered: that is a real result on a dead reverse link (RS-0.12
B1, 0/86), and the block says so loudly. Timestamps are not used to score:
the base and tractor clocks are not comparable.
"""
from __future__ import annotations

import argparse
import math
import os
import re

_BASE_MARK = re.compile(r"PROBE TX\b")
_BASE_TX = re.compile(r"PROBE TX seq=(\d+) phase_ms=(-?\d+) size_b=(\d+) \(tx#(\d+)\)")
_TR_MARK = re.compile(r"LoRa cmd: PROBE\b")
_TR_RX = re.compile(r"LoRa cmd: PROBE seq=(\d+) phase_ms=(-?\d+) \(rx#(\d+)\)")
# image_rx_daemon._send_command_frame logs every attempt; PROBE is opcode 0x69.
_ON_AIR = re.compile(r"command TX opcode=0x69 copy=\d+/\d+ (OK \(on air\)|FAILED)")
_MODE = re.compile(r"LIFETRAC_ENCODE_MODE=(\d+)")
# A daemon log line (Python logging's asctime): proof the file is a daemon log.
_TS_LINE = re.compile(r"^\ufeff?\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3} ", re.M)
CODEC_BY_MODE = {"9": "vector", "6": "mono_g4"}
# params reg_profile values whose legs pool (host_cfg_keys.h REG_PROFILE_*)
PROFILE_LABEL = {"1": "FHSS", "2": "DTS"}
ALPHA = 0.05
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
    seqs that the base sent), the tractor line count and duplicates, and the
    base's on-air / failed attempts for opcode 0x69 (None where the base log
    predates that line). Raises ScoreError on anything that would make the
    figure wrong rather than low, including a log that is not a daemon log
    (the tractor's is checked only when the base fired a probe)."""
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


def _pct(x: int, y: int) -> str:
    return f"{100.0 * x / y:.1f} %" if y else "n/a"


def format_leg(name: str, params: dict[str, str], res: dict) -> list[str]:
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
    return out


def pooled_verdict(vec: tuple[int, int], mono: tuple[int, int], profile: str = "2") -> list[str]:
    """vec / mono: (delivered, sent) pooled per codec, all legs on ``profile``
    (params reg_profile). DTS ("2") prints the VECTOR_SCENE 8.6 verdict; any
    other profile the same comparison, labelled with it."""
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
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("archive", nargs="+",
                    help="harness archive dir(s): radio_monitor_<stamp>_<sha>")
    a = ap.parse_args(argv)
    rc = 0
    pools: dict[tuple[str, str], list[int]] = {}     # (reg_profile, codec) -> [delivered, sent]
    unlike: list[str] = []                   # legs not shown to be FHSS / DTS with -ProbeEcho 0
    other: list[str] = []                    # legs that are neither VECTOR nor mono_g4
    scored: list[str] = []                   # every scored leg, with its profile and echo
    for arch in a.archive:
        name = os.path.basename(os.path.normpath(arch))
        try:
            params = parse_params(_read(os.path.join(arch, "params.txt")))
            res = score_leg(_read(os.path.join(arch, "rx_daemon.log")),
                            _read(os.path.join(arch, "tx_daemon.log")))
        except ScoreError as exc:
            print(f"{name}: CANNOT SCORE: {exc}")
            rc = 2
            continue
        if res["sent"] == 0:
            print(f"{name}: no PROBE TX line in rx_daemon.log (reactive_fire="
                  f"{params.get('reactive_fire', '?')}): this leg fired no probe, so it has "
                  f"no command-delivery score (DTS legs need -ReactiveFire 1)")
            rc = max(rc, 1)
            continue
        for line in format_leg(name, params, res):
            print(line)
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
    if len(a.archive) > 1:
        codecs = {c for _, c in pools}
        profiles = sorted({p for p, _ in pools})
        both = "vector" in codecs and "mono_g4" in codecs
        if rc:
            print("pooled verdict withheld: a leg above could not be scored")
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
        elif both:
            prof = profiles[0]
            for line in pooled_verdict(tuple(pools[(prof, "vector")]),
                                       tuple(pools[(prof, "mono_g4")]), prof):
                print(line)
        else:
            print("pooled: " + "; ".join(
                f"{PROFILE_LABEL.get(p, f'reg_profile={p}')} {c} {d}/{s} = {_pct(d, s)}"
                for (p, c), (d, s) in pools.items())
                + " -- no VECTOR_SCENE 8.6 comparison (it needs VECTOR and mono_g4 legs)")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
