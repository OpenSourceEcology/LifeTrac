import re, sys, os, datetime as dt, statistics
ROOT=sys.argv[1]
LEGS=[('2c_f1','095315'),('2b_f1','100249'),('2c_f2','101341'),('2b_f2','102247'),('2c_f3','103156'),('2b_f3','104056')]
TS=re.compile(r'^\ufeff?(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3})')
def ts(s): return dt.datetime.strptime(s,'%Y-%m-%d %H:%M:%S,%f').timestamp()
def linfit(xs,ys):
    n=len(xs); mx=sum(xs)/n; my=sum(ys)/n
    sxx=sum((x-mx)**2 for x in xs); sxy=sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    b=sxy/sxx; a=my-b*mx
    res=[y-(a+b*x) for x,y in zip(xs,ys)]
    return a,b,res
for tag,stamp in LEGS:
    d=os.path.join(ROOT,f'radio_monitor_20261024_{stamp}_d28bc2ca')
    last=None; rf=[]; pend=None; arr=[]
    for l in open(os.path.join(d,'rx_daemon.log'),encoding='utf-8-sig',errors='replace'):
        m=TS.match(l)
        if m: last=ts(m.group(1))
        mm=re.search(r'RFCO_PERTX.*hop_idx=(\d+) channel_idx=(\d+) epoch=(\d+)',l)
        if mm: pend=int(mm.group(3))*50+int(mm.group(1))
        if 'command TX' in l and 'OK (on air)' in l and pend is not None:
            rf.append((pend,last)); pend=None
        mm=re.search(r'frag_arrival: seq=(\d+) idx=\d+ total=\d+ fw_us=(\d+)',l)
        if mm: arr.append((last,int(mm.group(2))))
    xs=[a for a,t in rf]; ys=[t for a,t in rf]
    a,b,res=linfit(xs,ys)
    # base L072 fw_us clock vs base host clock (rate)
    a2,b2,res2=linfit([f/1e6 for t,f in arr],[t for t,f in arr]) if len(arr)>2 else (0,0,[0])
    print(f'{tag}: n={len(rf)} slot period in base-host time = {b*1000:.3f} ms ({(b/0.2-1)*1e6:+.0f} ppm); resid sd {statistics.pstdev(res)*1000:.1f} ms, range {min(res)*1000:.0f}..{max(res)*1000:.0f} ms;'
          f'  base L072 fw_us rate vs base host = {(b2-1)*1e6:+.0f} ppm (n={len(arr)})')
