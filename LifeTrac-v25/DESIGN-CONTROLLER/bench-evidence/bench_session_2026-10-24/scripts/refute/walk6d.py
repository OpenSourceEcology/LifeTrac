exec(open('walk6.py').read().split("for o in")[0])
def sim3(Ta,k,off_ms=14.0):
    # k = PC seconds per base-ms-second (base tick fast by (1-k))
    D=0.5*k; AT=30.0*k
    for i,(s,(t,sq,toa)) in enumerate(zip(slots,tx)):
        st=b+P*s+off_ms/1000; en=st+toa/1e6
        rel=st-Ta
        if rel<0: continue
        a=int(rel//AT)
        if a>3: break
        d=int((rel-AT*a)//D)
        if d>59: continue
        if ch(s//50,s%50)==d%50 and en<=Ta+AT*a+D*(d+1):
            return i+1,a,round(rel,2)
    return None
for ppm in (0,500,1000,1400,2000):
    k=1-ppm*1e-6
    row=[]
    for o in range(-100,101,10):
        r=sim3(lbt+o/1000,k)
        row.append('%+d:%s'%(o,'none' if r is None else '#%d/a%d'%(r[0],r[1])))
    print('base tick fast %4d ppm:'%ppm,' '.join(row))
