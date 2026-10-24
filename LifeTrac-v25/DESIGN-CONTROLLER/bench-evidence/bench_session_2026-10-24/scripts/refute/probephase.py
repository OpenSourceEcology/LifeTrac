exec(open('allegs.py').read().split("for arch,known in")[0])
import collections
def load2(arch):
    A=B+'/'+arch
    par=dict(l.strip().split('=',1) for l in open(A+'/params.txt',encoding='utf-8-sig') if '=' in l)
    boff=float(par['base_clock_offset_s']); toff=float(par['tractor_clock_offset_s'])
    lines=[l for l in open(A+'/rx_daemon.log',encoding='utf-8-sig').read().splitlines() if l.strip()]
    out=[];fa=None;pseq=None;pend=None
    for l in lines:
        m=re.match(r'(\S+ \S+) INFO image_rx_daemon: frag_arrival: seq=(\d+)',l)
        if m: fa=ts(m.group(1))-boff
        m=re.match(r'(\S+ \S+) INFO image_rx_daemon: PROBE TX seq=(\d+)',l)
        if m: pseq=int(m.group(2)); pfa=fa
        m=re.match(r'RFCO_PERTX.*hop_idx=(\d+) channel_idx=(\d+) epoch=(\d+)',l)
        if m: pend=int(m.group(3))*50+int(m.group(1))
        m=re.match(r'(\S+ \S+) INFO image_rx_daemon: command TX opcode=0x69 copy=1/1 OK',l)
        if m: out.append((pseq,pfa,ts(m.group(1))-boff,pend)); pend=None
    got=set(int(m.group(1)) for m in re.finditer(r'LoRa cmd: PROBE seq=(\d+)',open(A+'/tx_daemon.log',encoding='utf-8-sig').read()))
    return out,got
for arch in ['radio_monitor_20261024_095315_d28bc2ca','radio_monitor_20261024_100249_d28bc2ca','radio_monitor_20261024_101341_d28bc2ca','radio_monitor_20261024_102247_d28bc2ca','radio_monitor_20261024_103156_d28bc2ca','radio_monitor_20261024_104056_d28bc2ca']:
    tx,rx,probes,lbt=load(arch)
    if arch.endswith('104056_d28bc2ca'):
        P=0.199684; 
        starts=[t-toa/1e6 for t,s,toa in tx]
        known={264:667,293:740,346:872,358:902,494:1243}
        xs=list(known.values()); ys=[starts[k-1] for k in known]
        n=len(xs); mx=sum(xs)/n; my=sum(ys)/n
        P=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/sum((x-mx)**2 for x in xs); b=my-P*mx
    else:
        P,b,starts=fit(tx,probes)
    out,got=load2(arch)
    same=collections.Counter(); 
    rows=[]
    for pseq,pfa,ok,ps in out:
        if ps is None: continue
        fslot=math.floor((pfa-b)/P)
        okph=(ok-b)/P-ps
        kind='same' if ps==fslot else ('next' if ps==fslot+1 else 'other%+d'%(ps-fslot))
        same[(kind,pseq in got)]+=1
        rows.append((okph,kind,pseq in got,pseq))
    sm=[r for r in rows if r[1]=='same']
    print(arch[14:29],dict(same),'same-slot ok-phase max %.3f'%max(r[0] for r in sm) if sm else '', 'misses',[ (r[3],r[1],round(r[0],3)) for r in rows if not r[2]])
    # distribution of ok phase for same-slot probes, top 5
    print('   top same-slot ok-phase:',sorted([round(r[0],3) for r in sm])[-6:])
