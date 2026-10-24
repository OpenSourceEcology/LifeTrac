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
rx=[]
for line in open(A+'/rx_daemon.log',encoding='utf-8-sig'):
    m=re.match(r'(\S+ \S+) INFO image_rx_daemon: frag_arrival: seq=(\d+) ',line)
    if m: rx.append((ts(m.group(1))-boff,int(m.group(2))))
print('n tx',len(tx),'n rx',len(rx))
# tx start estimate = txdone log time - toa
starts=[t-toa/1e6 for t,s,toa in tx]
t0=starts[0]
# fit period: iterative slot assignment
P=0.2
for it in range(5):
    ns=[round((s-t0)/P) for s in starts]
    # least squares
    n=len(ns); mx=sum(ns)/n; my=sum(starts)/n
    P=sum((x-mx)*(y-my) for x,y in zip(ns,starts))/sum((x-mx)**2 for x in ns)
    b=my-P*mx
ns=[round((s-b)/P) for s in starts]
res=[s-(b+P*k) for s,k in zip(starts,ns)]
print('P=%.5f ms'%(P*1000),'res min/max ms %.1f %.1f'%(min(res)*1000,max(res)*1000))
# check first slot
print('frame1 slot',ns[0])
frames=[]
for i,((t,s,toa),k) in enumerate(zip(tx,ns)):
    frames.append((i+1,t,s,k,k//50,k%50,ch(k//50,k%50)))
# map rx to frames by time
for rt,rs in rx:
    best=min(frames,key=lambda f:abs(f[1]-rt))
    print('rx seq',rs,'PC %.3f'%rt,'-> frame #%d seq %d slot(%d,%d) ch %d dt %.3f'%(best[0],best[2],best[4],best[5],best[6],rt-best[1]))
import json
json.dump(frames,open('frames_%s.json'%A.rstrip('/').split('_')[-2],'w'))
