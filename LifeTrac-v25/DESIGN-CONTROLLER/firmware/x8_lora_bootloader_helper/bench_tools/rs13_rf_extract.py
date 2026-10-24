#!/usr/bin/env python3
"""rs13_rf_extract.py -- received level per leg (R0, RSSI/SNR, CRC errors) from an archive.

The range-edge procedure (RS13_RANGE_EDGE_LEG.md) reads the received level
from the base's 10-s `rx_rf:` windows in rx_daemon.log (the healthy-frame
RSSI/SNR instrument of RS-11.6; the corrupt population is the `crc_dump:`
lines). Per leg it records the median of the window medians, the minimum
and n, for RSSI and for SNR, plus the crc_dump count with its RSSI/SNR. R0
is the median over the 0 dB legs' medians (Round A: 2c, 2a, 2c', 2a';
Round C: C0v, C0m) and must sit in -65 to -75 dBm.

    py -3 rs13_rf_extract.py <archive_dir> [<archive_dir> ...] [--lines] [--path antenna]
    py -3 rs13_rf_extract.py <archive_dir> --r0 -67 --step 9 [--floor -114]
                             [--prev-snr 5.0 --prev-step 36]
    py -3 rs13_rf_extract.py --leak <archive_dir> [...]

--path conducted (the default) is the attenuator path the procedure was
written for. --path antenna is for legs flown on the antennas (2026-10-18
RS-12.15, 2026-10-19 Round A): the level prints, and over several archives
their median, but not the R0 verdict -- the -65 to -75 dBm window and its
pad advice belong to the conducted path. The tracking rule, the closing
check and the leakage check need the conducted path, so --path antenna
refuses --r0, --step, --closing and --leak.

Per archive: the window count and the frames in them, RSSI and SNR (median
of the window medians, minimum of the window medians, minimum of the window
minimums), the crc_dump count and RSSI/SNR spread, the published-frame count
and the FRF readback of both daemons. A DTS leg (reg_profile=2) is flagged
unless both boards read back `(OK)` at the pick (params force_frf_hz; 915 MHz
when it is 0 or absent, the helper's default) and no read-back is off it, a
MISMATCH or a failed force-write -- an abort condition of the procedure --
and flagged when it flew on 915 MHz by params or by read-back (A19). With
several archives: R0 over them
and the range check. --lines appends the per-window and per-dump rows (the
legs/leg<X>_rf.txt extract). --r0/--step (one archive) applies the walk's
RSSI tracking rule: within +/-3 dB of R0 - step while the median is >= 10 dB
above --floor (the pick's survey floor); below that, the median SNR must
fall 3 +/- 2 dB per 3 dB step from --prev-snr at --prev-step. --leak scores
the leakage check instead: no frame decoded (no `published frame_id`, no
fragment, no rx_rf window, rx_frames 0, with the stats thread reporting
`rx_frames=` to the end of the log), and the crc_dump RSSI is the leakage
level.

Exit 0 when every archive gave its figures (a tracking or R0-range miss is a
result, printed, not an error); 1 when an archive cannot give them (no rx_rf
window although frames were published, the stats thread or the receive worker
crashed, or the windows stop before the frames do -- RS-12.17; with --leak, no
`rx_frames=` line, a crashed stats thread, receive worker or receiver, stats
lines that stop before the
log does or cover less than the leg's duration_s (which params.txt must give
as a positive number), or a DTS leg without the
FRF read-back at the pick) or, with --leak, when a frame was decoded; 2 on
missing or garbled input. Only rx_daemon.log (one clock)
is read for timing.
"""
from __future__ import annotations

import argparse
import math
import os
import re
import statistics
from datetime import datetime

_TS = re.compile(r"^\ufeff?(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3}) ")
_NUM = r"-?\d+(?:\.\d+)?"
_RXRF = re.compile(
    rf"rx_rf: n_rssi=(\d+) n_snr=(\d+) rssi\[min/med/max\]=(None|{_NUM}/{_NUM}/{_NUM}) "
    rf"snr\[min/med/max\]=(None|{_NUM}/{_NUM}/{_NUM})\s*$")
_CRC = re.compile(rf"crc_dump: irq=0x[0-9a-fA-F]+ rx_len=(\d+) snr=({_NUM}) rssi=(-?\d+) dump=")
# method_h_stage2_tx_probe_v2.py: "(OK)" or "(**MISMATCH**)" after the read-back value.
_FRF = re.compile(r"FRF readback: (0x[0-9A-Fa-f]+) = ([\d.]+) MHz \(([^)]*)\)")
# The helper forces profile 2 to 915 MHz when LIFETRAC_FORCE_FRF_HZ is unset
# (-ForceFrfHz 0), and prints the read-back to 1 kHz.
DTS_DEFAULT_FRF_HZ = 915_000_000
FRF_TOL_HZ = 1000
R0_WINDOW = (-75.0, -65.0)
STATS_PERIOD_S = 10.0
# Frames may publish for up to one stats period after the last window before
# the log ends (the window in progress is never printed); allow 5 s more.
TAIL_SLACK_S = STATS_PERIOD_S + 5.0


class ExtractError(Exception):
    """Missing or garbled input (exit 2)."""


def _ts(line: str) -> float | None:
    m = _TS.match(line)
    if not m:
        return None
    return datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S,%f").timestamp()


def _triple(text: str):
    if text == "None":
        return None
    lo, med, hi = (float(x) for x in text.split("/"))
    return lo, med, hi


def parse_rx_log(text: str) -> dict:
    """The rx_rf windows, crc_dumps and frame events of one rx_daemon.log.

    Raises ExtractError on an rx_rf or crc_dump line that does not parse or
    whose fields contradict each other: a partial extract would be a wrong
    one."""
    windows, dumps, published, frags, rx_frames = [], [], [], 0, []
    t_first = t_last = t_rx_last = None
    for n, line in enumerate(text.splitlines(), 1):
        t = _ts(line)
        if t is not None and t_first is None:
            t_first = t
        if t is not None:
            t_last = t
        if "rx_rf:" in line:
            m = _RXRF.search(line)
            if not m or t is None:
                raise ExtractError(f"rx_daemon.log line {n}: rx_rf line does not parse: "
                                   f"{line.strip()[:160]!r}")
            n_rssi, n_snr = int(m.group(1)), int(m.group(2))
            rssi, snr = _triple(m.group(3)), _triple(m.group(4))
            for name, cnt, tri in (("rssi", n_rssi, rssi), ("snr", n_snr, snr)):
                if (cnt == 0) != (tri is None) or (tri and not tri[0] <= tri[1] <= tri[2]):
                    raise ExtractError(f"rx_daemon.log line {n}: rx_rf {name} fields "
                                       f"contradict (n={cnt}, {tri}): {line.strip()[:160]!r}")
            windows.append({"t": t, "n_rssi": n_rssi, "n_snr": n_snr, "rssi": rssi, "snr": snr})
        elif "crc_dump:" in line:
            m = _CRC.search(line)
            if not m or t is None:
                raise ExtractError(f"rx_daemon.log line {n}: crc_dump line does not parse: "
                                   f"{line.strip()[:160]!r}")
            dumps.append({"t": t, "rx_len": int(m.group(1)), "snr": float(m.group(2)),
                          "rssi": int(m.group(3))})
        elif "published frame_id" in line and t is not None:
            published.append(t)
        elif "frag_arrival" in line:
            frags += 1
        m = re.search(r"\brx_frames=(\d+)", line)
        if m:
            rx_frames.append(int(m.group(1)))
            if t is not None:
                t_rx_last = t
    if t_first is None:
        raise ExtractError("rx_daemon.log has no timestamped line")
    return {"windows": windows, "dumps": dumps, "published": published, "frags": frags,
            "rx_frames_max": max(rx_frames) if rx_frames else None, "t_first": t_first,
            "t_last": t_last, "t_rx_last": t_rx_last,
            "stats_crashed": "Exception in thread image-rx-stats" in text,
            # Reception runs in its own thread (image_rx_daemon: image-rx-worker); the
            # stats thread keeps printing zero counters after it dies.
            "worker_crashed": "Exception in thread image-rx-worker" in text,
            "rx_fatal": "fatal: cannot open L072 HostLink" in text}


def leak_unscorable(log: dict, params: dict | None = None) -> str | None:
    """Why a leg with nothing decoded still cannot PASS the leakage check, or None.

    No frame decoded is evidence only while the receiver ran and its stats
    thread reported rx_frames to the end of the log and across the leg."""
    if log["stats_crashed"]:
        return "the stats thread crashed (`Exception in thread image-rx-stats`)"
    if log["worker_crashed"]:
        return ("the receive worker crashed (`Exception in thread image-rx-worker`): reception "
                "stopped while the stats kept printing")
    if log["rx_fatal"]:
        return "the receiver never opened its L072 HostLink (`fatal: cannot open L072 HostLink`)"
    if log["rx_frames_max"] is None:
        return "no `rx_frames=` stats line: the receiver's counters never reported"
    gap = log["t_last"] - log["t_rx_last"] if log["t_rx_last"] is not None else None
    if gap is None or gap > TAIL_SLACK_S:
        return (f"the last `rx_frames=` stats line is {gap:.0f} s before the log ends "
                f"(> {TAIL_SLACK_S:.0f} s): the stats thread stopped (RS-12.17)"
                if gap is not None else "no timestamped `rx_frames=` stats line")
    raw = (params or {}).get("duration_s")
    try:
        dur = float(raw)
    except (TypeError, ValueError):
        dur = math.nan
    if not (math.isfinite(dur) and dur > 0):
        return (f"params duration_s={raw!r} is not a positive number of seconds: the receiver's "
                f"coverage of the leg cannot be checked")
    span = log["t_rx_last"] - log["t_first"]
    if span < dur - TAIL_SLACK_S:
        return (f"the `rx_frames=` stats lines cover {span:.0f} s of a {dur:.0f} s leg: the "
                f"receiver stopped early")
    return None


def _axis(windows: list, key: str, ncol: str) -> dict | None:
    rows = [w for w in windows if w[key] is not None]
    if not rows:
        return None
    meds = [w[key][1] for w in rows]
    return {"windows": len(rows), "samples": sum(w[ncol] for w in rows),
            "med": statistics.median(meds), "min_med": min(meds),
            "min": min(w[key][0] for w in rows), "max": max(w[key][2] for w in rows)}


def summarize(log: dict) -> dict:
    d = log["dumps"]

    def spread(key):
        v = [x[key] for x in d]
        return (statistics.median(v), min(v), max(v)) if v else None

    return {"rssi": _axis(log["windows"], "rssi", "n_rssi"),
            "snr": _axis(log["windows"], "snr", "n_snr"),
            "crc": len(d), "crc_rssi": spread("rssi"), "crc_snr": spread("snr")}


def unusable(log: dict) -> str | None:
    """Why the windows cannot stand for the leg, or None."""
    if log["stats_crashed"]:
        return ("the stats thread crashed (`Exception in thread image-rx-stats`): its rx_rf "
                "windows stop there and describe only the start of the leg (RS-12.17)")
    if log["worker_crashed"]:
        return ("the receive worker crashed (`Exception in thread image-rx-worker`): reception "
                "stopped there, so the windows describe only the start of the leg")
    pub = log["published"]
    if pub and not log["windows"]:
        return (f"{len(pub)} frames published but no rx_rf window: the base daemon predates "
                f"the RS-11.6 rx_rf instrument, or its stats thread never ran")
    if pub and log["windows"]:
        late = [t for t in pub if t > log["windows"][-1]["t"] + TAIL_SLACK_S]
        if late:
            return (f"the rx_rf windows stop {late[-1] - log['windows'][-1]['t']:.0f} s before "
                    f"the last published frame ({len(late)} frames after them): the stats "
                    f"thread stopped (RS-12.17)")
    return None


def frf_readbacks(text: str) -> tuple[list[tuple[float, str]], bool]:
    """``([(MHz, status)], force-write failed)`` of one daemon log; status is
    what the helper printed in the brackets (OK or **MISMATCH**)."""
    return ([(float(m.group(2)), m.group(3)) for m in _FRF.finditer(text)],
            "FRF force-write failed" in text)


def _frf_text(rbs: list[tuple[float, str]], failed: bool) -> str:
    out = [f"{mhz:.3f} MHz ({st})" for mhz, st in rbs]
    if failed:
        out.append("FRF force-write FAILED")
    return ", ".join(out) or "none"


def _pick_hz(params: dict) -> int | None:
    """The DTS carrier the leg asked for, or None when params garble it."""
    try:
        return int(params.get("force_frf_hz") or 0) or DTS_DEFAULT_FRF_HZ
    except ValueError:
        return None


def _at(mhz: float, hz: int) -> bool:
    return abs(mhz * 1e6 - hz) <= FRF_TOL_HZ


def frf_check(params: dict, base: tuple, trac: tuple | None) -> list[str]:
    """The `!!` lines for a DTS leg's FRF readbacks (empty when they hold).

    ``base`` / ``trac``: frf_readbacks() of each daemon log (``trac`` None
    without a tx_daemon.log). Holds when each board read back (OK) at the
    pick, and no read-back on either is off it or not OK, and no force-write
    failed."""
    if params.get("reg_profile") != "2":
        return []
    out = []
    pick = _pick_hz(params)
    boards = (("base", base), ("tractor", trac if trac is not None else ([], False)))
    if pick is None:
        out.append(f"  !! params force_frf_hz={params.get('force_frf_hz')!r} is not an integer: "
                   f"the pick is unknown, so `FRF readback: ... (OK)` at the pick cannot be "
                   f"shown -- an abort condition of the range-edge procedure")
    else:
        bad = [f"{name} {_frf_text(rbs, failed)}" for name, (rbs, failed) in boards
               if failed or not rbs
               or not all(st == "OK" and _at(mhz, pick) for mhz, st in rbs)]
        if bad:
            out.append(f"  !! DTS leg without `FRF readback: ... (OK)` at the pick "
                       f"{pick / 1e6:.3f} MHz on both boards ({'; '.join(bad)}): an abort "
                       f"condition of the range-edge procedure")
    by_params = ("params" if params.get("force_frf_hz") == str(DTS_DEFAULT_FRF_HZ) else
                 "params (force_frf_hz 0 or absent: the helper's default)")
    how = ([by_params] if pick == DTS_DEFAULT_FRF_HZ else []) + [
        f"the {name} read-back" for name, (rbs, _) in boards
        if any(st == "OK" and _at(mhz, DTS_DEFAULT_FRF_HZ) for mhz, st in rbs)]
    if how:
        out.append(f"  !! DTS leg on 915.000 MHz (by {' and '.join(how)}), the RS-11.6 emitter "
                   f"channel (RS-13.1 A19)")
    return out


def parse_params(text: str) -> dict[str, str]:
    out = {}
    for line in text.splitlines():
        key, sep, val = line.lstrip("\ufeff").partition("=")
        if sep:
            out[key.strip()] = val.strip()
    return out


def _read(path: str, required: bool = True) -> str | None:
    if not os.path.isfile(path):
        if required:
            raise ExtractError(f"missing {path}")
        return None
    with open(path, encoding="utf-8-sig", errors="replace") as fh:
        return fh.read()


def _fmt(x: float, unit: str, nd: int = 1) -> str:
    return f"{x:.{nd}f} {unit}"


def format_leg(name: str, params: dict, log: dict, s: dict, rx_text: str,
               tx_text: str | None, usable: bool = True, path: str = "conducted") -> list[str]:
    """The per-archive block. ``usable`` False (the windows do not stand for
    the leg) drops the one-line table row, so a stale figure is not copied.
    ``path`` antenna labels the header (the conducted header is unchanged)."""
    mode = re.search(r"LIFETRAC_ENCODE_MODE=(\d+)", params.get("cam_extra_env", ""))
    out = [f"{name}: reg_profile={params.get('reg_profile', '?')} "
           f"force_frf_hz={params.get('force_frf_hz', '?')} "
           f"encode_mode={mode.group(1) if mode else '-'} "
           f"duration_s={params.get('duration_s', '?')}"
           + (" path=antenna" if path == "antenna" else "")]
    w = log["windows"]
    if w:
        out.append(f"  rx_rf windows: {len(w)}, +{w[0]['t'] - log['t_first']:.0f}..+"
                   f"{w[-1]['t'] - log['t_first']:.0f} s from the log start")
    else:
        out.append("  rx_rf windows: 0 (no healthy frame received)")
    r, q = s["rssi"], s["snr"]
    for label, ax, unit, nd in (("RSSI:", r, "dBm", 0), ("SNR: ", q, "dB", 1)):
        if ax:
            out.append(f"  {label} median of window medians {_fmt(ax['med'], unit)}; min of window "
                       f"medians {_fmt(ax['min_med'], unit, nd)}; min of window mins "
                       f"{_fmt(ax['min'], unit, nd)}; max {_fmt(ax['max'], unit, nd)}; "
                       f"n = {ax['windows']} windows, {ax['samples']} frames")
    if s["crc"]:
        (rm, rl, rh), (sm, sl, sh) = s["crc_rssi"], s["crc_snr"]
        out.append(f"  crc_dump: {s['crc']} (RSSI med/min/max {rm:.0f}/{rl}/{rh} dBm; "
                   f"SNR med/min/max {sm:.1f}/{sl:.1f}/{sh:.1f} dB)")
    else:
        out.append("  crc_dump: 0")
    out.append(f"  published frame_id: {len(log['published'])}; frag_arrival: {log['frags']}; "
               f"rx_frames (last counter max): {log['rx_frames_max']}")
    if r and q and usable:
        out.append(f"  row: RSSI med / min {r['med']:.1f} / {r['min']:.0f} dBm | SNR med / min "
                   f"{q['med']:.1f} / {q['min']:.1f} dB | CRC err {s['crc']} | n {r['windows']}")
    base_frf = frf_readbacks(rx_text)
    trac_frf = frf_readbacks(tx_text) if tx_text is not None else None
    trac = _frf_text(*trac_frf) if trac_frf is not None else "no tx_daemon.log"
    out.append(f"  FRF readback: base {_frf_text(*base_frf)}; tractor {trac}")
    out += frf_check(params, base_frf, trac_frf)
    return out


def tracking(med_rssi: float, med_snr: float | None, r0: float, step: float,
             floor: float | None, prev_snr: float | None, prev_step: float | None) -> list[str]:
    """The walk's RSSI tracking rule for one leg (RS13_RANGE_EDGE_LEG.md)."""
    exp = r0 - step
    if floor is None or med_rssi >= floor + 10.0:
        d = med_rssi - exp
        where = ("no --floor given: assumed the RSSI region" if floor is None else
                 f"RSSI region: median >= floor {floor:.1f} + 10 dB")
        verdict = "HOLDS" if abs(d) <= 3.0 else "BROKEN"
        return [f"tracking: step {step:g} dB, expected R0 - step = {exp:.1f} dBm; median "
                f"{med_rssi:.1f} dBm (delta {d:+.1f} dB, limit +/-3) -> {verdict} ({where})"]
    head = (f"tracking: median RSSI {med_rssi:.1f} dBm is less than 10 dB above the floor "
            f"{floor:.1f} dBm: SNR region")
    if med_snr is None:
        return [head + "; no SNR window -> NO VERDICT"]
    if prev_snr is None or prev_step is None:
        return [head + f"; median SNR {med_snr:.1f} dB -- give --prev-snr and --prev-step to "
                       f"check the 3 +/- 2 dB fall per 3 dB step -> NO VERDICT"]
    fall = (prev_snr - med_snr) * 3.0 / (step - prev_step)
    verdict = "HOLDS" if 1.0 <= fall <= 5.0 else "BROKEN"
    return [head + f"; median SNR {med_snr:.1f} dB vs {prev_snr:.1f} dB at step {prev_step:g}: "
                   f"falls {fall:.1f} dB per 3 dB step (3 +/- 2) -> {verdict}"]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("archive", nargs="+",
                    help="harness archive dir(s): radio_monitor_<stamp>_<sha>")
    ap.add_argument("--lines", action="store_true",
                    help="append the per-window and per-dump rows (legs/leg<X>_rf.txt)")
    ap.add_argument("--leak", action="store_true",
                    help="score the leakage check (no frame decoded)")
    ap.add_argument("--r0", type=float, help="R0 in dBm, for the tracking rule (one archive)")
    ap.add_argument("--step", type=float,
                    help="the leg's nominal step in dB, for the tracking rule")
    ap.add_argument("--floor", type=float, help="the pick's survey floor in dBm")
    ap.add_argument("--prev-snr", type=float, help="the previous step's median SNR (SNR region)")
    ap.add_argument("--prev-step", type=float, help="the previous step in dB (SNR region)")
    ap.add_argument("--closing", action="store_true",
                    help="the closing check (B0close / C0close): median RSSI within 2 dB of --r0")
    ap.add_argument("--path", choices=("conducted", "antenna"), default="conducted",
                    help="the RF path the legs flew: conducted (default; R0's -65 to -75 dBm "
                         "verdict) or antenna (the level only)")
    a = ap.parse_args(argv)
    if a.path == "antenna" and (a.leak or a.closing or a.r0 is not None or a.step is not None):
        ap.error("--path antenna gives the level only: the leakage check, the tracking rule "
                 "(--r0 / --step) and the closing check need the conducted path")
    if a.closing and (a.r0 is None or a.step is not None):
        ap.error("--closing takes --r0 and no --step (the chain is back at 0 dB)")
    if not a.closing and (a.r0 is None) != (a.step is None):
        ap.error("--r0 and --step go together")
    if a.r0 is not None and (len(a.archive) != 1 or a.leak):
        ap.error("--r0 takes exactly one archive (and not --leak)")
    if a.step is None and any(x is not None for x in (a.floor, a.prev_snr, a.prev_step)):
        ap.error("--floor / --prev-snr / --prev-step need --r0 and --step")
    if (a.prev_snr is None) != (a.prev_step is None):
        ap.error("--prev-snr and --prev-step go together")
    if a.prev_step is not None and a.prev_step >= a.step:
        ap.error("--prev-step must be below --step")

    rc, medians, snr_meds, profiles = 0, [], [], set()
    for arch in a.archive:
        name = os.path.basename(os.path.normpath(arch))
        try:
            rx_text = _read(os.path.join(arch, "rx_daemon.log"))
            params = parse_params(_read(os.path.join(arch, "params.txt")))
            tx_text = _read(os.path.join(arch, "tx_daemon.log"), required=False)
            log = parse_rx_log(rx_text)
        except ExtractError as exc:
            print(f"{name}: CANNOT EXTRACT: {exc}")
            rc = 2
            continue
        s = summarize(log)
        why = None if a.leak else unusable(log)
        for line in format_leg(name, params, log, s, rx_text, tx_text, usable=why is None,
                               path=a.path):
            print(line)
        if a.leak:
            decoded = (len(log["published"]), log["frags"], len(log["windows"]),
                       log["rx_frames_max"] or 0)
            if any(decoded):
                print(f"  leakage check: FAIL -- decoded: published {decoded[0]}, frag_arrival "
                      f"{decoded[1]}, rx_rf windows {decoded[2]}, rx_frames {decoded[3]}")
                rc = max(rc, 1)
            elif leak_unscorable(log, params):
                print(f"  leakage check: NOT SCORED -- nothing decoded, but "
                      f"{leak_unscorable(log, params)}")
                rc = max(rc, 1)
            elif any("abort condition" in ln for ln in frf_check(
                    params, frf_readbacks(rx_text),
                    frf_readbacks(tx_text) if tx_text is not None else None)):
                print("  leakage check: NOT SCORED -- nothing decoded, but the boards did not "
                      "read back (OK) at the pick (the !! line above): the base may have "
                      "listened off the leg's carrier")
                rc = max(rc, 1)
            else:
                lvl = (f"leakage level (crc_dump RSSI) med/max {s['crc_rssi'][0]:.0f}/"
                       f"{s['crc_rssi'][2]} dBm" if s["crc"] else "no crc_dump line")
                print(f"  leakage check: PASS -- no frame decoded; {lvl}. Check link_stats "
                      f"rx_frames_seen 0 and an empty base capture too")
        else:
            if why:
                print(f"  !! NO LEVEL FOR THIS LEG: {why}")
                rc = max(rc, 1)
            elif s["rssi"] is None:
                print("  !! NO LEVEL FOR THIS LEG: no rx_rf RSSI window (no healthy frame "
                      "received; a leakage leg takes --leak)")
                rc = max(rc, 1)
            else:
                medians.append(s["rssi"]["med"])
                profiles.add(params.get("reg_profile", "?"))
                if s["snr"]:
                    snr_meds.append(s["snr"]["med"])
                if a.closing:
                    d = s["rssi"]["med"] - a.r0
                    verdict = "OK" if abs(d) <= 2.0 else "SUSPECT: a connector moved"
                    print(f"  closing check: median RSSI {s['rssi']['med']:.1f} dBm vs R0 "
                          f"{a.r0:.1f} dBm (delta {d:+.1f} dB, limit +/-2) -> {verdict}; "
                          f"compare its seq-gap loss with the 0 dB legs' too")
                elif a.r0 is not None:
                    for line in tracking(s["rssi"]["med"], s["snr"]["med"] if s["snr"] else None,
                                         a.r0, a.step, a.floor, a.prev_snr, a.prev_step):
                        print("  " + line)
        if a.lines:
            for w in log["windows"]:
                r = "/".join(f"{x:.0f}" for x in w["rssi"]) if w["rssi"] else "None"
                q = "/".join(f"{x:.1f}" for x in w["snr"]) if w["snr"] else "None"
                print(f"    rx_rf +{w['t'] - log['t_first']:7.1f} s n_rssi={w['n_rssi']:3d} "
                      f"n_snr={w['n_snr']:3d} rssi[min/med/max]={r} snr[min/med/max]={q}")
            for d in log["dumps"]:
                print(f"    crc_dump +{d['t'] - log['t_first']:7.1f} s rx_len={d['rx_len']} "
                      f"rssi={d['rssi']} snr={d['snr']:.1f}")
    if not a.leak and len(a.archive) > 1:
        what = "R0" if a.path == "conducted" else "level"
        if rc:
            print(f"{what} withheld: an archive above gave no level")
        elif len(profiles) > 1:
            print(f"{what} withheld: the legs mix reg_profile {' and '.join(sorted(profiles))}; "
                  + ("R0 is per profile (DTS from Round A, FHSS from C0v/C0m)"
                     if a.path == "conducted" else "the level is per profile"))
            rc = 2
        else:
            r0 = statistics.median(medians)
            snr = f" / SNR {statistics.median(snr_meds):.1f} dB" if snr_meds else ""
            if a.path == "antenna":
                print(f"level over {len(medians)} legs (median of the legs' medians): RSSI "
                      f"{r0:.1f} dBm{snr}; antenna path: the level only (R0's -65 to -75 dBm "
                      f"window and its pad advice belong to the conducted path)")
                return rc
            lo, hi = R0_WINDOW
            where = ("in range" if lo <= r0 <= hi else
                     "OUT OF RANGE: above -65 dBm (add 10 dB to the fixed middle, radios "
                     "parked, and re-measure)" if r0 > hi else
                     "OUT OF RANGE: below -75 dBm (take loss out of the fixed middle, radios "
                     "parked, and re-measure)")
            print(f"R0 over {len(medians)} legs (median of the legs' medians): RSSI {r0:.1f} dBm"
                  f"{snr}; target -65 to -75 dBm: {where}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
