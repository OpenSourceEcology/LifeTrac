# -*- coding: utf-8 -*-
"""Group the replay results (risk_res.pkl from risk_run.py) by workload."""
import pickle, statistics, sys, math
R = pickle.load(open(sys.argv[1], 'rb'))['a']
RATES = (0, 610, 1000, 1400)
groups = {
  '1 fps camera (1 fragment/frame)': lambda n, r: r['txs'] < 1.2,
  '~2 TX/s camera (2 fps, 1 fragment/frame)': lambda n, r: 1.2 <= r['txs'] < 2.2,
  '2.4-3.3 TX/s camera (multi-fragment frames)': lambda n, r: r['kind'] == 'camera' and r['txs'] >= 2.2,
  '>= 2.8 TX/s synth (multi-fragment)': lambda n, r: r['kind'] == 'synth',
  '2026-10-24 VECTOR (2b_f1-f3)': lambda n, r: '20261024' in n and n.split('_')[3] in ('100249', '102247', '104056'),
  '2026-10-24 mono_g4 (2c_f1-f3)': lambda n, r: '20261024' in n and n.split('_')[3] in ('095315', '101341', '103156'),
}
for lab, sel in groups.items():
    rs = [(n, r) for n, r in R.items() if sel(n, r)]
    fails = sum(1 for n, r in rs if r['t_obs'] > 120)
    ps = [[r['p'][i] for n, r in rs] for i in range(4)]
    print('%-46s n=%2d observed past 120 s: %d | model mean P(no catch) %s | per-leg range %s'
          % (lab, len(rs), fails,
             ' / '.join('%.1f%%' % (100 * statistics.mean(p)) for p in ps),
             '%.1f-%.1f%%' % (100 * min(min(p) for p in ps), 100 * max(max(p) for p in ps))))
    print('      legs: %s' % ', '.join('%s (%.2f TX/s, t_obs %.1f s)' % (n[14:29], r['txs'], r['t_obs']) for n, r in rs))
