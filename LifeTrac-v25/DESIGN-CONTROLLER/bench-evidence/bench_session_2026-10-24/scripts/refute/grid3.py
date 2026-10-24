import re,sys,datetime as dt,statistics
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
print('P from 5 catches = %.3f ms'%(P*1000), 'resid ms',[round((y-(b+P*x))*1000,1) for x,y in zip(xs,ys)])
ks=[(s-b)/P for s in starts]
fr=[k-int(k//1) for k in ks]
import collections
h=collections.Counter(int(f*20) for f in fr)
print('fractional slot phase histogram (x0.05):',sorted(h.items()))
import math
slots=[math.floor(k+0.5) for k in ks]
# check monotonic & unique
dup=[i for i in range(1,len(slots)) if slots[i]<=slots[i-1]]
print('non-increasing slot assignments:',len(dup))
for k in known: print('frame',k,'slot',slots[k-1],'expected',known[k])
chs=[ch(s//50,s%50) for s in slots]
def pc(t): return dt.datetime.fromtimestamp(t,dt.timezone.utc).strftime('%H:%M:%S.%f')[:-3]
print('slot0 PC',pc(b),'frame1 slot',slots[0],'last frame slot',slots[-1],pc(starts[-1]))
windows=[('ch9 from final FAIL ~15:37:38.5 to C1',9,None,667),('ch44 C1->C2',44,668,740),('ch6 C2->C3',6,741,872),('ch6 C3->C4',6,873,902),('ch18 C4->C5',18,903,1243),('ch18 after C5',18,1244,10**9)]
for name,c,lo,hi in windows:
    if lo is None:
        # time of 15:37:38.5 PC -> slot
        tfail=b+(667-((starts[263]-dt.datetime(2026,10,24,15,37,38,500000,tzinfo=dt.timezone.utc).timestamp())/P))*P
        lo=math.floor((dt.datetime(2026,10,24,15,37,38,500000,tzinfo=dt.timezone.utc).timestamp()-b)/P)
    hits=[(i+1,slots[i],(slots[i]//50,slots[i]%50),pc(starts[i])) for i in range(len(slots)) if chs[i]==c and lo<=slots[i]<=hi]
    print(name,'slots',lo,hi,'hits',hits)
# Also: frames on ch 9 between 15:37:13 and 15:37:38.5
lo=math.floor((dt.datetime(2026,10,24,15,37,0,tzinfo=dt.timezone.utc).timestamp()-b)/P)
print('ch9 frames from 15:37:00:',[(i+1,slots[i],pc(starts[i])) for i in range(len(slots)) if chs[i]==9 and slots[i]>=lo][:5])
