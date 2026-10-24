import pickle,os
GR=pickle.load(open('grid.pkl','rb'))
R=GR['2b_f3']; t0=R['t_lbt']; fr=R['frames']
print('leg6 grid P=%.3f ms slot0 LBT%+.3f s'%(R['P']*1000,R['anchor']-t0))
caught={264:'C1',293:'C2',346:'C3',358:'C4',494:'C5'}
# walker model at delta=+0.04: final FAIL at +120.04 on ch 9
for st,en,i,a,c,ins in fr:
    if i in caught: print(f'{caught[i]} idx {i} slot {a} ({a//50},{a%50}) ch {c} in-slot {ins*1000:.0f} ms +{st-t0:.2f}s')
park=[(120.04,9)]
probe=[(264,44),(293,6),(346,6),(358,18),(494,18)]
seq=[(120.04,9)]
for i,c in probe:
    t=[f[1] for f in fr if f[2]==i][0]-t0
    seq.append((t+0.05,c))
seq.append((400,None))
for k in range(len(seq)-1):
    ta,c=seq[k]; tb=seq[k+1][0]
    hits=[(f[2],round(f[0]-t0,2)) for f in fr if f[4]==c and ta<f[0]-t0<tb+0.3]
    print(f'parked on ch {c} from +{ta:.2f} to +{min(tb,320):.2f}: tractor frames on ch {c}: {hits}')
# list all frames on ch 9 near the end of the scan to bound when the base parked
print('ch9 frames +100..+140:',[(f[2],round(f[0]-t0,2),f[3]) for f in fr if f[4]==9 and 100<f[0]-t0<140])
