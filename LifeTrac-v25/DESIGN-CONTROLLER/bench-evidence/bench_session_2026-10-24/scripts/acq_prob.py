import pickle,os,statistics,collections
from acq_sim2 import sim, GR
import io,contextlib
res={}
for tag,R in GR.items():
    outs=[]
    for k in range(-500,501,2):   # delta -5..+5 s, 20 ms steps
        outs.append(sim(R,k*0.01,lat=0.006))
    fails=sum(1 for o in outs if o is None)
    att=collections.Counter(o[1] for o in outs if o)
    times=[o[3] for o in outs if o]
    res[tag]=(fails,len(outs),att,times)
    # per-attempt success prob ~ fraction acquiring in attempt k given reaching it
    reach=len(outs); per=[]
    for a in range(4):
        s=att.get(a,0); per.append(s/reach if reach else 0); reach-=s
    print(f'{tag}: P(no catch in 4x30 s) = {fails}/{len(outs)} = {100*fails/len(outs):.1f} %; acquired in attempt 0/1/2/3 = {[att.get(a,0) for a in range(4)]}; '
          f'per-attempt success given reached = {[round(x,2) for x in per]}; median t_acq {statistics.median(times):.1f} s')
v=[res[t] for t in ('2b_f1','2b_f2','2b_f3')]; m=[res[t] for t in ('2c_f1','2c_f2','2c_f3')]
print('VECTOR pooled fail: %d/%d = %.1f %%'%(sum(x[0] for x in v),sum(x[1] for x in v),100*sum(x[0] for x in v)/sum(x[1] for x in v)))
print('mono_g4 pooled fail: %d/%d = %.1f %%'%(sum(x[0] for x in m),sum(x[1] for x in m),100*sum(x[0] for x in m)/sum(x[1] for x in m)))
