import re,sys,datetime as dt,math,statistics
sys.path.insert(0,'.')
from perm import ch
B='/c/Users/dorkm/Documents/GitHub/LifeTrac-bench-main/LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence'.replace('/c/','C:/')
def ts(s):
    d=dt.datetime.strptime(s,'%Y-%m-%d %H:%M:%S,%f'); return d.replace(tzinfo=dt.timezone.utc).timestamp()
def load(arch):
    A=B+'/'+arch
    par=dict(l.strip().split('=',1) for l in open(A+'/params.txt',encoding='utf-8-sig') if '=' in l)
    toff=float(par['tractor_clock_offset_s']); boff=float(par['base_clock_offset_s'])
    tx=[]
    for line in open(A+'/tx_daemon.log',encoding='utf-8-sig'):
        m=re.match(r'(\S+ \S+) INFO image_tx_daemon: txdone_arrival: seq=(\d+) idx=(\d+) status=(\d+) toa_us=(\d+)',line)
        if m: tx.append((ts(m.group(1))-toff, int(m.group(2)), int(m.group(5))))
    lines=[l for l in open(A+'/rx_daemon.log',encoding='utf-8-sig').read().splitlines() if l.strip()]
    probes=[];rx=[];lbt=None;pend=None
    for l in lines:
        if 'LBT_ENABLE' in l and lbt is None: lbt=ts(l[:23])-boff
        m=re.match(r'(\S+ \S+) INFO image_rx_daemon: frag_arrival: seq=(\d+)',l)
        if m: rx.append((ts(m.group(1))-boff,int(m.group(2))))
        m=re.match(r'RFCO_PERTX.*hop_idx=(\d+) channel_idx=(\d+) epoch=(\d+)',l)
        if m: pend=(int(m.group(3))*50+int(m.group(1)),int(m.group(2)))
        m=re.match(r'(\S+ \S+) INFO image_rx_daemon: command TX opcode=0x69 copy=1/1 OK',l)
        if m and pend: probes.append((ts(m.group(1))-boff,)+pend); pend=None
    return tx,rx,probes,lbt
def fit(tx,probes):
    xs=[p[1] for p in probes]; ys=[p[0] for p in probes]
    n=len(xs); mx=sum(xs)/n; my=sum(ys)/n
    P=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/sum((x-mx)**2 for x in xs); b=my-P*mx
    # adjust b so tractor TX starts sit at phase ~0.05 (as in leg 6)
    starts=[t-toa/1e6 for t,s,toa in tx]
    ph=[((s-b)/P)%1 for s in starts]
    # circular mean
    import cmath
    c=sum(cmath.exp(2j*math.pi*p) for p in ph)/len(ph); mph=(cmath.phase(c)/(2*math.pi))%1
    dph=((mph-0.05+0.5)%1)-0.5
    b2=b+P*dph
    # sanity: the probe check slot channel matches
    return P,b2,starts
def sim(tx,slots,P,b,Ta,off_ms=14.0):
    for i,(s,(t,sq,toa)) in enumerate(zip(slots,tx)):
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
for arch,known in [('radio_monitor_20261024_095315_d28bc2ca',None),('radio_monitor_20261024_100249_d28bc2ca',None),('radio_monitor_20261024_101341_d28bc2ca',None),('radio_monitor_20261024_102247_d28bc2ca',None),('radio_monitor_20261024_103156_d28bc2ca',None)]:
    tx,rx,probes,lbt=load(arch)
    P,b,starts=fit(tx,probes)
    slots=[math.floor((s-b)/P+0.5) for s in starts]
    nonmono=sum(1 for i in range(1,len(slots)) if slots[i]<=slots[i-1])
    # first rx frame -> tractor frame by time
    first=rx[0][0]; fi=min(range(len(tx)),key=lambda i:abs(tx[i][0]-first))
    # check probe channel consistency: probe slot channel equals RFCO channel always by construction; check frame channel of rx frames vs following-probe? skip
    row=[]
    for o in range(-60,121,10):
        r=sim(tx,slots,P,b,lbt+o/1000)
        row.append('%+d:%s'%(o,'none' if r is None else '#%d'%(r[0])))
    print(arch[14:29],'P=%.3fms nonmono=%d first rx = frame #%d (+%.1fs after LBT), slot %d'%(P*1000,nonmono,fi+1,first-lbt,slots[fi]))
    print('   ',' '.join(row))
