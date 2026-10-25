# -*- coding: utf-8 -*-
"""Load + grid-fit every archived profile-1 leg; pickle for the model runs.

Cold-start risk and clock analysis for RS-12.21 / FHSS leg 6 (2b_f3),
2026-10-24, no radio. Design memo:
firmware/murata_l072/FHSS_SCAN_FAILED_OPTIONS.md. Run from this folder (the
bench-evidence root is ..), with the pickles outside the repo:
  py -3 load_all.py .. <tmp>/legs.pkl       > load_all.txt
  py -3 risk_run.py <tmp>/legs.pkl          > risk_run.txt     (writes <tmp>/risk_res.pkl)
  py -3 risk_summary.py <tmp>/risk_res.pkl  > risk_summary.txt
  py -3 clock_run.py <tmp>/legs.pkl         > clock_run.txt
Needs numpy. fhss_perm.py is a copy of
../bench_session_2026-10-24/scripts/fhss_perm.py (the firmware hop
permutation, checked there against every logged RFCO_PERTX).
Usage: py -3 load_all.py <bench-evidence root> <out.pkl>"""
import os
import pickle
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from legload import survey, load, grid_fit  # noqa: E402

root, out = sys.argv[1], sys.argv[2]
legs = []
for d in survey(root):
    L = grid_fit(load(d))
    rx = L['rx']
    first = rx[0][0] - L['t_blbt']                      # base host, as survey.py
    fi = L['match'][0]
    nm = sum(1 for m in L['match'] if m is not None)
    print('%s fps=%s feed=%s tx=%d rx=%d matched=%d off=%.3f (%s, first-lines %.3f) '
          'P=%.3f ms (%+.0f ppm) score %d/%d first rx +%.1f s frame #%s slot %s ch %s anchor_line=%s num=%s %s'
          % (L['name'], L['fps'], L['feed'], len(L['tx']), len(rx), nm, L['off'], L['off_src'],
             L['off0'], L['P'] * 1000, L['ppm_grid'], L['grid_score'], len(L['tx']), first,
             None if fi is None else fi + 1, None if fi is None else int(L['slot'][fi]),
             None if fi is None else int(L['ch'][fi]), L['anchor_line'], L['num_src'], L['num_agree']))
    legs.append(L)
pickle.dump(legs, open(out, 'wb'))
print(len(legs), 'legs')
