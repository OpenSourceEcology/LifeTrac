import re,sys,datetime as dt
B='C:/Users/dorkm/Documents/GitHub/LifeTrac-bench-main/LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence'
def ts(s): return dt.datetime.strptime(s,'%Y-%m-%d %H:%M:%S,%f').timestamp()
for d in ['095315','100249','101341','102247','103156','104056']:
    A=B+'/radio_monitor_20261024_%s_d28bc2ca'%d
    pts=[]
    for l in open(A+'/rx_daemon.log',encoding='utf-8-sig'):
        m=re.match(r'(\S+ \S+) INFO image_rx_daemon: frag_arrival: seq=\d+ idx=\d+ total=\d+ fw_us=(\d+)',l)
        if m: pts.append((ts(m.group(1)),int(m.group(2))/1e6))
    if len(pts)<2: continue
    # unwrap fw_us (u32 us wraps at 4294.967296 s)
    un=[];off=0;prev=None
    for h,f in pts:
        if prev is not None and f<prev-1000: off+=4294.967296
        un.append((h,f+off)); prev=f
    n=len(un); mx=sum(h for h,f in un)/n; my=sum(f for h,f in un)/n
    s=sum((h-mx)*(f-my) for h,f in un)/sum((h-mx)**2 for h,f in un)
    # also tractor tx fit period from P -> handled elsewhere
    print(d,'n',n,'d(fw)/d(host) = %.6f -> fw clock %+.0f ppm vs base host'%(s,(s-1)*1e6))
