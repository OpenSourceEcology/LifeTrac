import re,sys,datetime as dt,math
sys.path.insert(0,'.')
from perm import ch
A=sys.argv[1]; toff=float(sys.argv[2]); boff=float(sys.argv[3])
def ts(s):
    d=dt.datetime.strptime(s,'%Y-%m-%d %H:%M:%S,%f'); return d.replace(tzinfo=dt.timezone.utc).timestamp()
tx=[]
for line in open(A+'/tx_daemon.log',encoding='utf-8-sig'):
    m=re.match(r'(\S+ \S+) INFO image_tx_daemon: txdone_arrival: seq=(\d+) idx=(\d+) status=(\d+) toa_us=(\d+)',line)
    if m: tx.append((ts(m.group(1))-toff, int(m.group(2)), int(m.group(5))))
starts=[t-toa/1e6 for t,s,toa in tx]
known={264:667,293:740,346:872,358:902,494:1243}
xs=[known[k] for k in known]; ys=[starts[k-1] for k in known]
n=len(xs); mx=sum(xs)/n; my=sum(ys)/n
P=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/sum((x-mx)**2 for x in xs); b=my-P*mx
slots=[math.floor((s-b)/P+0.5) for s in starts]
# on-air: start = b + P*slot + off_ms, dur = toa
lbt=None
for line in open(A+'/rx_daemon.log',encoding='utf-8-sig'):
    if 'LBT_ENABLE' in line: lbt=ts(line[:23])-boff
print('base LBT PC',lbt)
def sim(Ta,off_ms=14.0):
    for i,(s,(t,sq,toa)) in enumerate(zip(slots,tx)):
        st=b+P*s+off_ms/1000; en=st+toa/1e6
        rel=st-Ta
        if rel<0: continue
        a=int(rel//30)
        if a>3: break
        d=int((rel-30*a)//0.5)
        if d>59: continue
        wch=d%50
        dw_end=Ta+30*a+0.5*(d+1)
        if ch(s//50,s%50)==wch and en<=dw_end:
            return i+1,a,round(rel,2)
    return None
for o in [x/100 for x in range(-100,101,5)]:
    print('offset %+.2f'%o, sim(lbt+o))
