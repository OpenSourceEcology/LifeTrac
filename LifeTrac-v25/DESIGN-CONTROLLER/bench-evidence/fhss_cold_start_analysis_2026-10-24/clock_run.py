# -*- coding: utf-8 -*-
"""Clock side item: how fast does the base L072 tick run against the tractor
L072 (the two HSI16 clocks), measured from committed evidence only?

Per leg (all 27 profile-1 legs that log frag_arrival):
  r_bh  base L072 fw_us vs base host log time            (fwrate.py / period_fit.py)
  r_th  tractor L072 vs tractor host: the fitted slot period P is 200 tractor-ms
        in tractor-host time, so r_th = 0.2/P - 1
  r_bt  base L072 vs TRACTOR L072, two ways:
        (1) fw_us of each matched catch regressed on the tractor-host END time
            of the same frame (st + toa), times P/0.2;
        (2) fw_us regressed on the slot-grid time 0.2*slot + 13.4 ms + toa
            (pure L072-to-L072, no host clock), Theil-Sen.
Then leg 6 (2b_f3): the PROBE labels vs the drift of the base's free-running
adopted clock since its last SNAPPED catch.
Usage: py -3 clock_run.py legs.pkl
"""
import os
import pickle
import random
import re
import statistics
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from legload import ts, TS, chan  # noqa: E402

legs = pickle.load(open(sys.argv[1], 'rb'))


def unwrap_us(vals):
    out = []
    off = 0
    prev = None
    for v in vals:
        if prev is not None and v < prev - 2 ** 31:
            off += 2 ** 32
        prev = v
        out.append((v + off) / 1e6)
    return np.array(out)


def linfit(x, y):
    A = np.vstack([x - x.mean(), np.ones_like(x)]).T
    b, a = np.linalg.lstsq(A, y - 0, rcond=None)[0]
    res = y - (a + b * (x - x.mean()))
    return b, a, res


def theil_sen(x, y, n=4000, seed=1):
    rnd = random.Random(seed)
    m = len(x)
    sl = []
    for _ in range(n):
        i, j = rnd.randrange(m), rnd.randrange(m)
        if abs(x[j] - x[i]) > 5.0:
            sl.append((y[j] - y[i]) / (x[j] - x[i]))
    return statistics.median(sl) if sl else float('nan')


rows = []
print('leg              n_rx  r_bh(base L072 vs base host)  r_th(tractor L072 vs host)  r_bt(1) r_bt(2)   [ppm; negative = base L072 slower]')
for L in legs:
    rx = L['rx']
    fw = unwrap_us([r[3] for r in rx])
    th = np.array([r[0] for r in rx])
    r_bh = (linfit(th, fw)[0] - 1) * 1e6 if len(rx) > 2 else float('nan')
    r_th = (0.2 / L['P'] - 1) * 1e6
    idx = [(k, m) for k, m in enumerate(L['match']) if m is not None]
    if len(idx) >= 3:
        fwm = np.array([fw[k] for k, m in idx])
        en = np.array([L['en'][m] for k, m in idx])
        s1 = linfit(en, fwm)[0]
        r1 = (s1 * L['P'] / 0.2 - 1) * 1e6
        xs = np.array([0.2 * int(L['slot'][m]) + 0.0134 + float(L['toa'][m]) for k, m in idx])
        s2 = theil_sen(list(xs), list(fwm))
        r2 = (s2 - 1) * 1e6
    else:
        r1 = r2 = float('nan')
    rows.append((L['name'], len(rx), r_bh, r_th, r1, r2))
    print('%-15s  %5d  %+8.0f  %+8.0f  %+8.0f %+8.0f' % (L['name'][14:29], len(rx), r_bh, r_th, r1, r2))
r_bh = [r[2] for r in rows]
r_th = [r[3] for r in rows]
r1 = [r[4] for r in rows]
print('r_bh range %+.0f..%+.0f ppm (median %+.0f); r_th range %+.0f..%+.0f (median %+.0f); '
      'r_bt(1) range %+.0f..%+.0f (median %+.0f)'
      % (min(r_bh), max(r_bh), statistics.median(r_bh), min(r_th), max(r_th), statistics.median(r_th),
         min(r1), max(r1), statistics.median(r1)))
s24 = [r for r in rows if '20261024' in r[0]]
print('2026-10-24 legs: r_bh %s; r_th %s; r_bt(1) %s; r_bt(2) %s' % tuple(
    ['%+.0f..%+.0f' % (min(v), max(v)) for v in ([r[i] for r in s24] for i in (2, 3, 4, 5))]))

# ---------------------------------------------------------------- leg 6 PROBEs
# Leg 6 (2b_f3): do the five PROBE labels and the heard / lost outcome fit a
# base clock that free-runs between SNAPPED catches at the measured HSI16
# rate, or a clock that does not drift (the +-2 ppm TCXO assumption)?
#   catch k RX_DONE (tractor in-slot ms) = 14 (header slot_offset; the base's
#     `phase_telemetry ... slot_off=[14..14]`) + the frame's ToA (tractor log);
#   PROBE key-up = that + (OK log - frag_arrival log, base host) - PROBE ToA
#     + B (+-J per PROBE): B absorbs unequal URC/log latencies, scanned;
#   firmware gate (sx1276_tx.c sx1276_tx_slot_wait_us): the base keys only
#     when its clock's in-slot reading R has 12 <= R and R + ToA + 15 <= 200;
#   the base clock lags the tractor grid by -rate x (time since the last
#     SNAPPED catch); SNAPPED/EPOCH_DRIFT by the scheduler-epoch rule
#     (catch 1 UNANCHORED, then +-1 epoch of the last own TX / SNAP);
#   heard = the PROBE lies wholly inside the tractor slot it is labelled
#     with (the tractor follows its own clock and retunes at its boundary).
print()
L = [x for x in legs if '104056' in x['name']][0]
lines = list(open(os.path.join(L['dir'], 'rx_daemon.log'), encoding='utf-8-sig', errors='replace'))
ev = []
last = None
for l in lines:
    m = TS.match(l)
    if m:
        last = ts(m.group(1))
    if 'frag_arrival' in l:
        mm = re.search(r'seq=(\d+) .*fw_us=(\d+) len=(\d+)', l)
        ev.append(['catch', last, int(mm.group(2)), int(mm.group(3))])
    if 'RFCO_PERTX' in l:
        mm = re.search(r'hop_idx=(\d+) channel_idx=(\d+) epoch=(\d+).*pkt_toa_us=(\d+)', l)
        ev[-1].append((int(mm.group(3)), int(mm.group(1)), int(mm.group(2)), int(mm.group(4))))
    if 'command TX' in l and 'OK (on air)' in l:
        ev[-1].append(last)
heard_obs = [True, True, False, True, True]   # tx_daemon.log:1135, 1257, -, 1525, 2085 (RESULTS.md)
catches = []
for k, e in enumerate(ev):
    m = L['match'][k]
    catches.append(dict(t_frag=e[1], fw=e[2] / 1e6, slot=int(L['slot'][m]), toa=float(L['toa'][m]),
                        frame=m + 1, probe=e[4], t_ok=e[5], ch=int(L['ch'][m]), heard=heard_obs[k]))
x = np.array([0.2 * c['slot'] + 0.0134 + c['toa'] for c in catches])
y = np.array([c['fw'] for c in catches])
b, a, res = linfit(x, y)
r6 = (b - 1) * 1e6
print('leg 6: base fw_us vs tractor slot-grid time over the 5 catches: %+.0f ppm (residuals %s ms)'
      % (r6, ', '.join('%+.1f' % (v * 1000) for v in res)))
sched_epoch = 0
anchor_t = None
for i, c in enumerate(catches):
    if i == 0 or abs(c['slot'] // 50 - sched_epoch) <= 1:
        c['dec'], anchor_t = 'SNAPPED', c['fw']
    else:
        c['dec'] = 'EPOCH_DRIFT'
    c['age'] = c['fw'] - anchor_t
    pe, ph, pch, ptoa = c['probe']
    c['ptoa'] = ptoa / 1000.0
    c['lab'] = 'same' if pe * 50 + ph == c['slot'] else ('next' if pe * 50 + ph == c['slot'] + 1 else 'other')
    c['lat'] = (c['t_ok'] - c['t_frag']) * 1000.0
    sched_epoch = pe                                   # an own TX moves the scheduler
    print('catch %d: frame #%d slot (%d,%d) ch %d ToA %.1f ms, %s, base clock age %.1f s; PROBE (%d,%d) ch %d = %s slot, '
          '%s; OK - frag log %.0f ms'
          % (i + 1, c['frame'], c['slot'] // 50, c['slot'] % 50, c['ch'], c['toa'] * 1000, c['dec'], c['age'],
             pe, ph, pch, c['lab'], 'heard' if c['heard'] else 'NOT heard', c['lat']))


def one(c, B, rate):
    K = 14.0 + c['toa'] * 1000 + c['lat'] + B - c['ptoa']
    R = K + rate * 1e-6 * c['age'] * 1000.0
    if 12 <= R <= 200 - c['ptoa'] - 15:
        lab, ls = 'same', 0
    elif 212 <= R <= 400 - c['ptoa'] - 15:
        lab, ls = 'next', 1
    else:
        return None
    return lab, (ls * 200 <= K and K + c['ptoa'] <= ls * 200 + 200)


def fits(rate, B, J):
    return all(any(one(c, B + j / 4.0, rate) == (c['lab'], c['heard'])
                   for j in range(int(-4 * J), int(4 * J) + 1)) for c in catches)


r24 = sorted(r[4] for r in s24)
for J in (1.0, 2.0, 3.0):
    out = []
    for name, rate in (('no drift (4 ppm)', -4.0), ('leg-6 fit', r6), ('10-24 smallest', r24[-1]), ('10-24 largest', r24[0])):
        Bs = [B / 2.0 for B in range(0, 41) if fits(rate, B / 2.0, J)]
        out.append('%s %+.0f ppm: B %s' % (name, rate, ('%.1f-%.1f ms' % (min(Bs), max(Bs))) if Bs else 'none in 0-20 ms'))
    rs = [r for r in range(-5000, 1001, 25) if any(fits(r, B / 2.0, J) for B in range(0, 41))]
    print('PROBE labels + outcomes, jitter +-%.0f ms: %s; rates that fit some B: %s'
          % (J, '; '.join(out), ('%+d..%+d ppm' % (min(rs), max(rs))) if rs else 'none'))
for c in catches:
    if c['dec'] == 'EPOCH_DRIFT':
        B = 11.0
        K = 14.0 + c['toa'] * 1000 + c['lat'] + B - c['ptoa']
        lag = -r6 * 1e-6 * c['age'] * 1000
        print('  PROBE after catch at (%d,%d), clock age %.1f s: lag %.0f ms at %+.0f ppm; keyed at tractor in-slot %.0f ms '
              '(B = 11 ms), base clock reads %.0f ms; ends at %.0f ms (slot boundary 200)'
              % (c['slot'] // 50, c['slot'] % 50, c['age'], lag, r6, K, K - lag, K + c['ptoa']))
print('Free-run drift at the measured base-vs-tractor rates: 10-24 legs %.1f-%.1f ms/s, archive max %.1f ms/s; '
      'two +-2 ppm TCXOs: <= 0.004 ms/s' % (-r24[-1] / 1000, -r24[0] / 1000, max(abs(v) for v in r1) / 1000))
for h, lab in ((0.5, '2 fps frame interval'), (1.0, 'authority streak gap'), (2.0, 'LOCK_LOSS_MS = CLOCK_FRESH_MS')):
    print('  over %.1f s (%s): %.1f-%.1f ms (10-24), %.1f ms (archive max), %.3f ms (TCXO)'
          % (h, lab, -r24[-1] * 1e-3 * h, -r24[0] * 1e-3 * h, max(abs(v) for v in r1) * 1e-3 * h, 4e-3 * h))
for ms, lab in ((4.0, '12 ms head start minus the ~8 ms retune + skew budget (sx1276_fhss_clock.h)'),
                (15.0, 'TX fit guard'), (100.0, 'half a slot')):
    print('  %5.1f ms (%s) reached after %.1f-%.1f s (10-24), %.1f s (archive max), %.0f s (TCXO)'
          % (ms, lab, ms / (-r24[0] * 1e-3), ms / (-r24[-1] * 1e-3), ms / (max(abs(v) for v in r1) * 1e-3), ms / 4e-3))
