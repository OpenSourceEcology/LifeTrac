exec(open('walk6.py').read().split("for o in")[0])
import random
def sim2(Ta,toa_fn,off_ms=14.0):
    for i,(s,(t,sq,toa)) in enumerate(zip(slots,tx)):
        toa=toa_fn(i,toa)
        st=b+P*s+off_ms/1000; en=st+toa/1e6
        rel=st-Ta
        if rel<0: continue
        a=int(rel//30)
        if a>3: break
        d=int((rel-30*a)//0.5)
        if d>59: continue
        if ch(s//50,s%50)==d%50 and en<=Ta+30*a+0.5*(d+1):
            return i+1,a,round(rel,2)
    return None
for lab,fn in [('actual VECTOR toa',lambda i,t:t),('const 130.7 ms (mono median)',lambda i,t:130688),('const 169 ms',lambda i,t:169088)]:
    row=[]
    for o in range(-60,61,10):
        r=sim2(lbt+o/1000,fn)
        row.append('%+d:%s'%(o,'none' if r is None else '#%d@%.1f'%(r[0],r[2])))
    print(lab); print('  ',' '.join(row))
