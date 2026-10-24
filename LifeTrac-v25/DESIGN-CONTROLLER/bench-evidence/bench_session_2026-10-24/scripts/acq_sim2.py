import pickle,os,sys
GR=pickle.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'grid.pkl'),'rb'))
def sim(R,delta,dwell=0.5,attempt=30.0,settle=0.0015,attempts=4,lat=0.006):
    """lat: txdone host-log latency already inside frame times (start/end shifted late) -> shift back"""
    t0=R['t_lbt']+delta
    for k in range(attempts):
        base=t0+attempt*k
        n=0
        while True:
            ds=base+n*dwell; de=ds+dwell
            if ds>=base+attempt: break
            c=n%50
            for st,en,i,a,ch,ins in R['frames']:
                s=st-lat; e=en-lat
                if ch==c and s>=ds+settle and e<=de and e<=base+attempt:
                    return (i,k,n,round(s-R['t_lbt'],2))
            n+=1
    return None
def intervals(xs,step=0.01):
    out=[];s=None;p=None
    for x in xs:
        if s is None: s=p=x
        elif abs(x-p-step)<1e-6: p=x
        else: out.append((s,p)); s=p=x
    if s is not None: out.append((s,p))
    return out
obs={t:min(R['match'].items(),key=lambda kv:kv[1])[0] for t,R in GR.items()}
for lat in (0.0,0.006,0.012):
    print('### assumed txdone log latency',lat*1000,'ms')
    for tag,R in GR.items():
        g=[];preds={}
        for k in range(-200,101):
            dl=round(k*0.01,2); r=sim(R,dl,lat=lat)
            preds[dl]=r
            if tag=='2b_f3':
                if r is None: g.append(dl)
            elif r and r[0]==obs[tag]: g.append(dl)
        lab='NO catch in 4x30 s' if tag=='2b_f3' else f'first catch == observed idx {obs[tag]}'
        print(f'  {tag}: delta in [-2,+1] s where model gives {lab}: {intervals(g)}')
        if tag=='2b_f3':
            print('     leg-6 predicted first catch at selected deltas:',{d:preds[d] for d in (-0.5,-0.3,-0.2,-0.1,-0.05,0.0,0.05,0.1,0.2)})
