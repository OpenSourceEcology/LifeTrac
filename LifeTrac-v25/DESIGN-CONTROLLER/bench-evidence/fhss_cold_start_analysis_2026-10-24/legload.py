# -*- coding: utf-8 -*-
"""Shared loader for the archived profile-1 legs (walker-model risk sizing).

Adapted from bench_session_2026-10-24/scripts/{grid_fit,acq_sim2,first_frames}.py
and refute/{survey,allegs}.py: paths are arguments, every profile-1 archive
that logs frag_arrival is loaded the same way, and the cross-board clock
offset comes from params.txt when it exists (2026-10-24) or else from
matching the base's frag_arrival (seq, idx) to the tractor's txdone_arrival.

Times: everything is mapped into TRACTOR HOST seconds.
"""
import datetime as dt
import glob
import math
import os
import re
import statistics

import numpy as np

from fhss_perm import perm

TS = re.compile(r'^\ufeff?(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3})')
TXD = re.compile(r'txdone_arrival: seq=(\d+) idx=(\d+) status=(\d+) toa_us=(\d+)')
FRA = re.compile(r'frag_arrival: seq=(\d+) idx=(\d+) total=(\d+) fw_us=(\d+) len=(\d+)')
LAT = 0.006   # txdone host-log latency (acq_prob.py uses 6 ms)

_P = {}


def chan(abs_slot):
    e = abs_slot // 50
    if e not in _P:
        _P[e] = perm(e)
    return _P[e][abs_slot % 50]


def ts(s):
    return dt.datetime.strptime(s, '%Y-%m-%d %H:%M:%S,%f').replace(
        tzinfo=dt.timezone.utc).timestamp()


def params(d):
    p = {}
    for l in open(os.path.join(d, 'params.txt'), encoding='utf-8-sig', errors='replace'):
        if '=' in l:
            k, v = l.strip().split('=', 1)
            p[k] = v
    return p


def survey(root):
    out = []
    for p in sorted(glob.glob(os.path.join(root, '**', 'params.txt'), recursive=True)):
        d = os.path.dirname(p)
        try:
            par = params(d)
        except Exception:
            continue
        if par.get('reg_profile') != '1':
            continue
        rx = os.path.join(d, 'rx_daemon.log')
        tx = os.path.join(d, 'tx_daemon.log')
        if not (os.path.exists(rx) and os.path.exists(tx)):
            continue
        if 'frag_arrival' not in open(rx, encoding='utf-8-sig', errors='replace').read():
            continue
        out.append(d)
    return out


def _log(path):
    """yield (last_ts, line); last_ts = latest timestamp seen (untimed lines inherit it)."""
    last = None
    for l in open(path, encoding='utf-8-sig', errors='replace'):
        m = TS.match(l)
        if m:
            last = ts(m.group(1))
        yield last, l


def load(d):
    par = params(d)
    L = dict(dir=d, name=os.path.basename(d), par=par)
    tx = []
    t_tlbt = t_tfirst = None
    anchor_line = False
    for t, l in _log(os.path.join(d, 'tx_daemon.log')):
        if t_tfirst is None and t is not None:
            t_tfirst = t
        if t_tlbt is None and 'LBT_ENABLE' in l:
            t_tlbt = t
        if (not anchor_line and not tx and 'RFCO_PERTX' in l
                and 'hop_idx=0 ' in l and 'epoch=0 ' in l):
            anchor_line = True
        m = TXD.search(l)
        if m and TS.match(l):
            if int(m.group(3)) != 0:
                continue
            tx.append((t, int(m.group(1)), int(m.group(2)), int(m.group(4)) / 1e6))
    rx = []
    rf = []          # base TX (PROBE/command) on air: (base host t, abs slot)
    pend = None
    t_blbt = t_bfirst = None
    for t, l in _log(os.path.join(d, 'rx_daemon.log')):
        if t_bfirst is None and t is not None:
            t_bfirst = t
        if t_blbt is None and 'LBT_ENABLE' in l:
            t_blbt = t
        m = FRA.search(l)
        if m and TS.match(l):
            rx.append((t, int(m.group(1)), int(m.group(2)), int(m.group(4)), int(m.group(5))))
        mm = re.search(r'RFCO_PERTX.*tx_status=0x00.*hop_idx=(\d+) channel_idx=(\d+) epoch=(\d+)', l)
        if mm:
            pend = int(mm.group(3)) * 50 + int(mm.group(1))
        if 'command TX' in l and 'OK (on air)' in l and pend is not None and TS.match(l):
            rf.append((t, pend))
            pend = None
    L['rf'] = rf
    L.update(tx=tx, rx=rx, t_tlbt=t_tlbt, t_blbt=t_blbt, anchor_line=anchor_line,
             fps=par.get('synth_fps'), feed=par.get('tx_feed', '?'))
    # ---- cross-board offset: base_host = tractor_host + off --------------------
    if 'base_clock_offset_s' in par and 'tractor_clock_offset_s' in par:
        off0 = float(par['base_clock_offset_s']) - float(par['tractor_clock_offset_s'])
        src = 'params'
    else:
        off0 = t_bfirst - t_tfirst
        src = 'first-lines'
    by_key = {}
    for i, (t, s, ix, toa) in enumerate(tx):
        by_key.setdefault((s, ix), []).append(i)
    diffs = []
    for (t, s, ix, fw, ln) in rx:
        for i in by_key.get((s, ix), ()):
            dd = t - tx[i][0]
            if abs(dd - off0) < 20.0:
                diffs.append(dd)
    if not diffs:
        raise RuntimeError('no seq match in ' + d)
    h = {}
    for dd in diffs:
        h[round(dd / 0.05)] = h.get(round(dd / 0.05), 0) + 1
    mode = max(h, key=h.get) * 0.05
    near = [dd for dd in diffs if abs(dd - mode) < 0.3]
    off = statistics.median(near)
    L.update(off=off, off_src=src, off0=off0)
    # match each base arrival to its tractor frame index
    match = []
    for (t, s, ix, fw, ln) in rx:
        best = None
        for i in by_key.get((s, ix), ()):
            dd = abs(t - off - tx[i][0])
            if dd < 0.3 and (best is None or dd < best[0]):
                best = (dd, i)
        match.append(None if best is None else best[1])
    L['match'] = match
    return L


def grid_fit(L):
    """Fit the tractor slot grid (period P, boundary phase) from its own TX
    starts, as grid_fit.py does (densest 16 ms phase window, boundary =
    window start - 12 ms), vectorised; then number slots."""
    tx = L['tx']
    st = np.array([t - toa - LAT for t, s, ix, toa in tx])
    t0 = st[0]
    best = None
    for coarse in (True, False):
        if coarse:
            ppms = np.arange(-6000, 2001, 10)
        else:
            c = best[1]
            ppms = np.arange(c - 10, c + 10.5, 0.5)
        for ppm in ppms:
            P = 0.2 * (1 + ppm * 1e-6)
            ph = np.mod(st - t0, P)
            bins = np.bincount(np.minimum((ph / 0.002).astype(int), 99), minlength=100)
            ext = np.concatenate([bins, bins[:8]])
            win = np.convolve(ext, np.ones(8, dtype=int), 'valid')[:100]
            j = int(np.argmax(win))
            sc = int(win[j])
            if best is None or sc > best[0]:
                best = (sc, ppm, j, P)
    sc, ppm, j, P = best
    phase0 = t0 + j * 0.002 - 0.012
    k = np.floor((st - phase0) / P).astype(int)
    # numbering: slot 0 = the tractor's anchor TX (lazy anchor at its first
    # FHSS TX). Its time is not logged; on 2026-10-24 the first image frame
    # was slot 8 when the anchor RFCO line precedes it (grid_fit.py, base
    # RFCO), so use k0 = k[0] - 8 then, else the first frame is slot 0.
    k0 = int(k[0]) - (8 if L['anchor_line'] else 0)
    num_src = 'first-frame'
    num_agree = None
    # Better, where the base logged its own PROBE/command TX slots (RFCO_PERTX
    # + "OK (on air)"): the mode of (base label - fitted grid index), as
    # grid_fit.py numbers the 2026-10-24 legs.
    if len(L['rf']) >= 3:
        cands = []
        for t_ok, a_b in L['rf']:
            k_rel = math.floor((t_ok - L['off'] - 0.03 - phase0) / P)
            cands.append(a_b - k_rel)
        nshift = statistics.mode(cands)
        num_agree = (sum(1 for c in cands if c == nshift), len(cands))
        k0 = -nshift
        num_src = 'base-RFCO'
    a = k - k0
    toa = np.array([toa for t, s, ix, toa in tx])
    ch = np.array([chan(int(x)) for x in a])
    L.update(num_src=num_src, num_agree=num_agree)
    L.update(P=P, ppm_grid=ppm, grid_score=sc, phase0=phase0, k0=k0,
             st=st, en=st + toa, toa=toa, slot=a, ch=ch,
             inslot=(st - phase0) - k * P)
    return L
