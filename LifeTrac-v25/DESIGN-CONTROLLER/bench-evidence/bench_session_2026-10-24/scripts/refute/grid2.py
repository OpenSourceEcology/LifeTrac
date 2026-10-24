import re,sys,datetime as dt
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
known={1:0,264:667,293:740,346:872,358:902,494:1243}
xs=[known[k] for k in known]; ys=[starts[k-1] for k in known]
n=len(xs); mx=sum(xs)/n; my=sum(ys)/n
P=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/sum((x-mx)**2 for x in xs); b=my-P*mx
print('P from known = %.4f ms'%(P*1000), 'resid ms',[round((y-(b+P*x))*1000,1) for x,y in zip(xs,ys)])
# refine with all frames using floor assignment of start (start within slot after boundary)
for it in range(3):
    ks=[round((s-b)/P - 0.25) for s in starts]  # offset guess
    offs=[(s-(b+P*k))*1000 for s,k in zip(starts,ks)]
print('offset in slot ms: min %.1f max %.1f'%(min(offs),max(offs)))
ks=[int((s-b)/P + 0.5) if False else None for s in starts]
# assign slot = floor((s - b + margin)/P) where margin chosen from offs distribution
import statistics
med=statistics.median(offs)
ks=[round((s-b)/P - med/1000/P) for s in starts]
offs=[(s-(b+P*k))*1000 for s,k in zip(starts,ks)]
print('median offset %.1f, after: min %.1f max %.1f'%(med,min(offs),max(offs)))
for k in known: print(k, ks[k-1], known[k])
frames=[(i+1,starts[i],ks[i]//50,ks[i]%50,ch(ks[i]//50,ks[i]%50)) for i in range(len(starts))]
import json; json.dump({'P':P,'b':b,'frames':frames},open('grid_leg6.json','w'))
# parked channel timeline
def find(t):
    return t
seq=[('final FAIL est',None)]
# list frames on chans 9,44,6,18 after start
for c in (9,44,6,18):
    print('ch',c,[(f[0],round(f[1]-starts[0],1),(f[2],f[3])) for f in frames if f[4]==c])
