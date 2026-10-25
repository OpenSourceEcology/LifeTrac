# -*- coding: utf-8 -*-
"""Walker-model cold-start risk over the archived profile-1 legs.

Model (acq_sim2.py / refute/allegs3.py, vectorised): the base's cold-start
walker dwells 500 ms per channel on channels 0,1,2,... (index mod 50), 60
dwells per 30 s attempt, back to channel 0 at each of the 4 attempts; a
frame is caught when it is on the dwelled channel and lies wholly inside the
dwell (start >= dwell start + 1.5 ms settle, end <= dwell end). The first
catch is the LOCK (a follower's first accepted header is UNANCHORED, so any
epoch is SNAPPED and SCANNING+FRAME_VALID -> LOCK). No catch in 4 x 30 s =
scan FAILED (absorbing on main). The base L072 ms tick runs `ppm` slow, so
every dwell and attempt lasts (1 - ppm*1e-6)^-1 times longer in host time.

Runs:
  A  replay   : each leg's own recorded tractor TX schedule; base activation
                swept -5..+5 s (20 ms steps) around the logged one.
  B  synthetic: one fragment per frame at 1, 2 and 5 fps on each leg's own
                fitted grid, ToA drawn from the leg's own TX ToAs, frames
                requested every 1/fps s +-40 ms and keyed by the firmware
                slot gate (13.4 ms into the slot, or later in the slot if
                ToA + 15 ms guard still fits); 4 schedule draws x 250
                activation offsets (40 ms steps over -5..+5 s).
  B' synthetic, stream already on: activation 20-30 s after the first frame
                (the SIL E2 definition).
Usage: py -3 risk_run.py legs.pkl
"""
import math
import os
import pickle
import statistics
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from legload import chan  # noqa: E402

SETTLE = 0.0015
RATES = (0, 610, 1000, 1400)          # base L072 tick, ppm SLOW vs host
FPS = (1, 2, 5)
legs = pickle.load(open(sys.argv[1], 'rb'))


def first_catch(st, en, ch, Ta, ppm):
    k = 1.0 / (1.0 - ppm * 1e-6)
    D = 0.5 * k
    AT = 30.0 * k
    rel = st - Ta
    idx = np.nonzero((rel >= 0) & (rel < 4 * AT))[0]
    if idx.size == 0:
        return None
    r = rel[idx]
    a = np.floor(r / AT)
    d = np.floor((r - a * AT) / D)
    ds = Ta + a * AT + d * D
    ok = ((d <= 59) & (np.mod(d, 50) == ch[idx]) & (st[idx] >= ds + SETTLE)
          & (en[idx] <= ds + D))
    j = int(np.argmax(ok))
    if not ok[j]:
        return None
    i = int(idx[j])
    return i, float(en[i] - Ta), int(a[j])


def synth(L, fps, rng, t0, t_end):
    P = L['P']
    ph = L['phase0']
    k0 = L['k0']
    toas = L['toa']
    sc = P / 0.2                      # host s per tractor s
    head = 0.0134 * sc
    st, en, ch = [], [], []
    busy = -1e18
    n = 0
    while True:
        treq = t0 + n / fps + rng.uniform(-0.04, 0.04)
        n += 1
        if treq > t_end:
            break
        toa = float(toas[rng.integers(len(toas))])
        tk = max(treq + 0.001, busy)
        for _ in range(4):
            kk = math.floor((tk - ph) / P)
            b = ph + kk * P
            ins = (tk - b) / sc       # tractor seconds into the slot
            if ins < 0.0134:
                tk = b + head
                ins = 0.0134
            if ins + toa + 0.015 <= 0.2 or ins <= 0.0135:
                break
            tk = b + P + head
        kk = math.floor((tk - ph) / P)
        st.append(tk)
        en.append(tk + toa)
        ch.append(chan(kk - k0))
        busy = tk + toa + 0.002
    return np.array(st), np.array(en), np.array(ch)


def wilson(x, n, z=1.96):
    p = x / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (c - h) / d, (c + h) / d


def clopper(x, n):
    # exact 95 % via bisection on the binomial tails
    from math import comb

    def cdf(k, p):
        return sum(comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(0, k + 1))
    lo = 0.0
    if x > 0:
        a, b = 0.0, 1.0
        for _ in range(60):
            m = (a + b) / 2
            if 1 - cdf(x - 1, m) < 0.025:
                a = m
            else:
                b = m
        lo = a
    a, b = 0.0, 1.0
    for _ in range(60):
        m = (a + b) / 2
        if cdf(x, m) > 0.025:
            a = m
        else:
            b = m
    return lo, a


def label(L):
    n = L['name'].replace('radio_monitor_', '')
    return n[:15]


def kind(L):
    if L['feed'] == 'local':
        return 'synth'
    return 'camera'


DELTAS_A = np.arange(-5.0, 5.0, 0.02)
DELTAS_B = np.arange(-5.0, 5.0, 0.04)
res_a = {}
print('=== A. replay of each leg (its own TX schedule), activation swept -5..+5 s ===')
print('leg              kind    fps  tx/s q    t_obs  | P(no catch <=120 s) at base tick 0 / -610 / -1000 / -1400 ppm | median t_catch | P(t<=t_obs)')
for L in legs:
    Ta0 = L['t_blbt'] - L['off']
    st, en, ch = L['st'], L['en'], L['ch']
    t_obs = L['rx'][0][0] - L['t_blbt']
    win = (st >= Ta0) & (st < Ta0 + 120)
    rate_tx = win.sum() / 120.0
    slots = set(L['slot'][win].tolist())
    q = len(slots) / (120.0 / L['P'])
    row = []
    meds = []
    pit = None
    for ppm in RATES:
        outs = [first_catch(st, en, ch, Ta0 + dl, ppm) for dl in DELTAS_A]
        fails = sum(1 for o in outs if o is None)
        row.append(fails / len(outs))
        times = [o[1] for o in outs if o]
        meds.append(statistics.median(times) if times else float('nan'))
        if ppm == 1000:
            pit = sum(1 for o in outs if o and o[1] <= t_obs + 0.05) / len(outs)
    res_a[L['name']] = dict(p=row, kind=kind(L), fps=L['fps'], t_obs=t_obs, med=meds, pit=pit,
                            txs=rate_tx, q=q)
    print('%-15s  %-6s  %-3s  %.2f %.2f %6.1f | %s | %s | %.2f'
          % (label(L), kind(L), L['fps'], rate_tx, q, t_obs,
             ' '.join('%5.1f%%' % (100 * p) for p in row),
             ' '.join('%5.1f' % m for m in meds), pit))
print()
for lab, sel in (('all 27', lambda r: True), ('camera (1 fragment/frame)', lambda r: r['kind'] == 'camera'),
                 ('synth (multi-fragment)', lambda r: r['kind'] == 'synth')):
    rs = [r for r in res_a.values() if sel(r)]
    for i, ppm in enumerate(RATES):
        ps = [r['p'][i] for r in rs]
        exp = sum(ps)
        # Poisson-binomial: P(0 failures) and P(>=1)
        p0 = 1.0
        for p in ps:
            p0 *= (1 - p)
        p1 = sum(p * np.prod([1 - q for j, q in enumerate(ps) if j != k]) for k, p in enumerate(ps))
        print('A %-26s n=%2d  -%4d ppm: mean P(no catch <=120 s) %5.2f %%, expected legs %4.2f, '
              'P(0 legs) %.2f, P(1 leg) %.2f, P(>=1) %.2f'
              % (lab, len(rs), ppm, 100 * statistics.mean(ps), exp, p0, p1, 1 - p0))
pits = [r['pit'] for r in res_a.values() if r['t_obs'] <= 120]
print('A calibration (-1000 ppm): P_model(t_catch <= t_obs) over the 26 legs that caught by 120 s: '
      'mean %.2f, quartiles %s' % (statistics.mean(pits), [round(x, 2) for x in statistics.quantiles(pits, n=4)]))
leg6 = [r for n, r in res_a.items() if '104056' in n][0]
print('A leg 6 (2b_f3): P(no catch) %s' % ['%.1f%%' % (100 * p) for p in leg6['p']])
lo, hi = clopper(1, 27)
print('Empirical: 1 of 27 legs past 120 s = %.1f %% (exact 95 %% CI %.2f-%.1f %%)' % (100 / 27, 100 * lo, 100 * hi))
print()

print('=== B. synthetic one-fragment-per-frame streams on each leg grid, harness timing ===')
rng = np.random.default_rng(20261024)
res_b = {}
for fps in FPS:
    for L in legs:
        Ta0 = L['t_blbt'] - L['off']
        t0 = float(L['st'][0])
        cnt = np.zeros(len(RATES))
        tot = 0
        cnt_on = np.zeros(len(RATES))
        tot_on = 0
        for rep in range(4):
            st, en, ch = synth(L, fps, rng, t0, Ta0 + 160 + 40)
            for i, ppm in enumerate(RATES):
                for dl in DELTAS_B:
                    if first_catch(st, en, ch, Ta0 + dl, ppm) is None:
                        cnt[i] += 1
                    # B': activation 20..30 s after the first frame
                    if first_catch(st, en, ch, t0 + 25.0 + dl, ppm) is None:
                        cnt_on[i] += 1
            tot += len(DELTAS_B)
            tot_on += len(DELTAS_B)
        res_b[(fps, L['name'])] = dict(p=cnt / tot, p_on=cnt_on / tot_on, kind=kind(L),
                                       toa_med=float(np.median(L['toa'])))
    for lab, sel in (('all 27', lambda r: True), ('camera ToA', lambda r: r['kind'] == 'camera'),
                     ('synth ToA', lambda r: r['kind'] == 'synth')):
        rs = [r for (f, n), r in res_b.items() if f == fps and sel(r)]
        for i, ppm in enumerate(RATES):
            ps = [r['p'][i] for r in rs]
            pon = [r['p_on'][i] for r in rs]
            print('B %d fps %-11s n=%2d -%4d ppm: P(no LOCK <=120 s) mean %5.2f %% (legs %4.1f-%4.1f %%) | '
                  'stream already on: mean %5.2f %% (legs %4.1f-%4.1f %%)'
                  % (fps, lab, len(rs), ppm, 100 * statistics.mean(ps), 100 * min(ps), 100 * max(ps),
                     100 * statistics.mean(pon), 100 * min(pon), 100 * max(pon)))
    print()
pickle.dump(dict(a=res_a, b=res_b), open(os.path.join(os.path.dirname(os.path.abspath(sys.argv[1])), 'risk_res.pkl'), 'wb'))
