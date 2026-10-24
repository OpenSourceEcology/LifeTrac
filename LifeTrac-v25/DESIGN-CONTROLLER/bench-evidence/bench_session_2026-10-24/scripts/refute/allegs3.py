exec(open('allegs.py').read().split("for arch,known in")[0])
def sim3(tx,slots,P,b,Ta,k,off_ms=14.0):
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
obs={'095315':139,'100249':57,'101341':91,'102247':57,'103156':19,'104056':None}
legs={}
for d in obs:
    arch='radio_monitor_20261024_%s_d28bc2ca'%d
    tx,rx,probes,lbt=load(arch)
    if d=='104056':
        starts=[t-toa/1e6 for t,s,toa in tx]
        known={264:667,293:740,346:872,358:902,494:1243}
        xs=list(known.values()); ys=[starts[kk-1] for kk in known]
        n=len(xs); mx=sum(xs)/n; my=sum(ys)/n
        P=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/sum((x-mx)**2 for x in xs); b=my-P*mx
    else:
        P,b,starts=fit(tx,probes)
    slots=[math.floor((s-b)/P+0.5) for s in starts]
    legs[d]=(tx,slots,P,b,lbt)
for ppm in (0,-600,-800,-1000,-1400):
    k=1-ppm*1e-6
    ok=[]
    for o in range(-150,151,5):
        good=True
        for d,want in obs.items():
            tx,slots,P,b,lbt=legs[d]
            r=sim3(tx,slots,P,b,lbt+o/1000,k)
            got=None if r is None else r[0]
            if d=='104056': continue
            if got!=want: good=False;break
        r6=sim3(*legs['104056'][:4],legs['104056'][4]+o/1000,k)
        if good: ok.append((o,'none' if r6 is None else '#%d'%r6[0]))
    print('base tick fast %d ppm: offsets (ms) reproducing legs 1-5 -> leg-6 prediction:'%ppm, ok)
