"""Fit the tractor's FHSS slot grid (period P, phase) from its own TX times (txdone_arrival - toa),
then number slots using the base-decoded (epoch,hop) of known caught frames. Times in BASE host time."""
import re, sys, os, datetime as dt, statistics, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fhss_perm import perm
ROOT=sys.argv[1]
LEGS=[('2c_f1','095315'),('2b_f1','100249'),('2c_f2','101341'),('2b_f2','102247'),('2c_f3','103156'),('2b_f3','104056')]
TS=re.compile(r'^\ufeff?(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3})')
def ts(s): return dt.datetime.strptime(s,'%Y-%m-%d %H:%M:%S,%f').timestamp()
P={}
def chan(a):
    e=a//50
    if e not in P: P[e]=perm(e)
    return P[e][a%50]
GR={}
for tag,stamp in LEGS:
    d=os.path.join(ROOT,f'radio_monitor_20261024_{stamp}_d28bc2ca')
    p={}
    for l in open(os.path.join(d,'params.txt'),encoding='utf-8-sig'):
        if '=' in l: k,v=l.strip().split('=',1); p[k]=v
    off0=float(p['base_clock_offset_s'])-float(p['tractor_clock_offset_s'])
    last=None; t_lbt=None; arr=[]; rf=[]; pend=None; pt=[]
    for l in open(os.path.join(d,'rx_daemon.log'),encoding='utf-8-sig',errors='replace'):
        m=TS.match(l)
        if m: last=ts(m.group(1))
        if 'LBT_ENABLE' in l and t_lbt is None: t_lbt=last
        mm=re.search(r'frag_arrival: seq=(\d+) idx=\d+ total=\d+ fw_us=(\d+) len=(\d+)',l)
        if mm: arr.append((last,int(mm.group(1))))
        mm=re.search(r'RFCO_PERTX.*hop_idx=(\d+) channel_idx=(\d+) epoch=(\d+)',l)
        if mm: pend=int(mm.group(3))*50+int(mm.group(1))
        if 'command TX' in l and 'OK (on air)' in l and pend is not None:
            rf.append((last,pend)); pend=None
    tx=[]; w=0; prev=-1
    for l in open(os.path.join(d,'tx_daemon.log'),encoding='utf-8-sig',errors='replace'):
        m=TS.match(l); mm=re.search(r'txdone_arrival: seq=(\d+) idx=\d+ status=(\d+) toa_us=(\d+)',l)
        if m and mm:
            s=int(mm.group(1))
            if s<prev: w+=1
            prev=s
            tx.append([ts(m.group(1))+off0,w*256+s,s,int(mm.group(3))/1e6])
    # empirical offset correction via matched seqs
    diffs=[]; match={}
    for ta,s in arr:
        c=[(abs(ta-t),t,i) for t,i,ss,toa in tx if ss==s]
        if c:
            dd,t,i=min(c)
            if dd<1.0: diffs.append(ta-t); match.setdefault(i,ta)
    corr=statistics.median(diffs)
    for x in tx: x[0]+=corr
    starts=[(te-toa,i,toa) for te,i,s,toa in tx]
    # grid search P, phase: score = #frames whose in-slot start in [8,24] ms
    best=None
    t0=starts[0][0]
    for k in range(-1200,401):
        Pp=0.2*(1+k*1e-6*5)   # step 5 ppm
        ph=[((st-t0)%Pp) for st,i,toa in starts]
        # histogram phase in 2 ms bins to find densest 16 ms window
        bins=[0]*100
        for x in ph: bins[min(99,int(x/0.002))]+=1
        sc=max(sum(bins[(j+q)%100] for q in range(8)) for j in range(100))
        if best is None or sc>best[0]:
            jbest=max(range(100),key=lambda j:sum(bins[(j+q)%100] for q in range(8)))
            best=(sc,Pp,jbest)
    sc,Pp,jb=best
    # slot boundary = window start - 12 ms
    phase0=t0+jb*0.002-0.012
    # integer numbering: use matched caught frames' slots from the base's first RFCO in the same slot? use RFCO lines: base TX slot a_b at time t_ok; find integer n so that grid slot index at (t_ok-0.03) maps to a_b
    cands=[]
    for t_ok,a_b in rf:
        k_rel=math.floor((t_ok-0.03-phase0)/Pp)
        cands.append(a_b-k_rel)
    nshift=statistics.mode(cands)
    agree=sum(1 for c in cands if c==nshift)
    anchor=phase0-nshift*Pp   # time of slot 0 start
    frames=[]
    for st,i,toa in starts:
        a=math.floor((st-anchor)/Pp)
        frames.append((st,st+toa,i,a,chan(a),(st-anchor)-a*Pp))
    GR[tag]=dict(t_lbt=t_lbt,anchor=anchor,P=Pp,frames=frames,match=match,first=arr[0] if arr else None)
    inslot=[f[5]*1000 for f in frames]
    print(f'{tag}: P={Pp*1000:.3f} ms ({(Pp/0.2-1)*1e6:+.0f} ppm) score {sc}/{len(starts)} in a 16 ms window; slot numbering agreement {agree}/{len(cands)}; '
          f'slot0 at LBT{anchor-t_lbt:+.3f}s; in-slot start median {statistics.median(inslot):.1f} ms')
    # verify catches: frame index of first catch & its channel vs base RFCO first
    fi=min(match.items(),key=lambda kv:kv[1])
    fr=[f for f in frames if f[2]==fi[0]][0]
    print(f'     first catch idx {fi[0]} at LBT+{fi[1]-t_lbt:.2f}s slot {fr[3]} ({fr[3]//50},{fr[3]%50}) ch {fr[4]}; first base RFCO slot {rf[0][1]} ({rf[0][1]//50},{rf[0][1]%50})')
import pickle
pickle.dump(GR,open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'grid.pkl'),'wb'))
