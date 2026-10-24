exec(open('walk6.py').read().split("for o in")[0])
import re
lines=open(A+'/rx_daemon.log',encoding='utf-8-sig').read().splitlines()
lines=[l for l in lines if l.strip()]
ev=[]
for i,l in enumerate(lines):
    m=re.match(r'(\S+ \S+) INFO image_rx_daemon: frag_arrival: seq=(\d+)',l)
    if m: fa=ts(m.group(1))-boff
    m=re.match(r'RFCO_PERTX.*hop_idx=(\d+) channel_idx=(\d+) epoch=(\d+)',l)
    if m: hop,chn,ep=map(int,m.groups())
    m=re.match(r'(\S+ \S+) INFO image_rx_daemon: command TX opcode=0x69 copy=1/1 OK',l)
    if m:
        ok=ts(m.group(1))-boff
        ev.append((fa,ok,ep,hop,chn))
# tractor PROBE log times
tp=[]
for l in open(A+'/tx_daemon.log',encoding='utf-8-sig'):
    m=re.match(r'(\S+ \S+) INFO image_tx_daemon: LoRa cmd: PROBE seq=(\d+)',l)
    if m: tp.append((int(m.group(2)),ts(m.group(1))-toff))
print('tractor PROBE logs',[(s,round(t-b,3)) for s,t in tp])
for k,(fa,ok,ep,hop,chn) in enumerate(ev):
    slot_abs=ep*50+hop
    # tractor slot at TX-done (ok) time
    tslot=(ok-b)/P
    print('PROBE%d base-TX slot (%d,%d) ch%d | frag_arr in tractor slot %.3f | cmd-OK log in tractor slot %.3f | slot boundary of base TX slot ends at %.3f'%(k+1,ep,hop,chn,(fa-b)/P,tslot,slot_abs+1))
