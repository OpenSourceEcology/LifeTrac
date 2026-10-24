import os,re,sys,datetime as dt,glob
ROOT=sys.argv[1]
TS=re.compile(r'^\ufeff?(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3})')
def ts(s): return dt.datetime.strptime(s,'%Y-%m-%d %H:%M:%S,%f').timestamp()
rows=[]
for p in glob.glob(os.path.join(ROOT,'**','params.txt'),recursive=True):
    d=os.path.dirname(p)
    rx=os.path.join(d,'rx_daemon.log')
    if not os.path.exists(rx): continue
    try: txt=open(p,encoding='utf-8-sig',errors='replace').read()
    except: continue
    m=re.search(r'reg_profile=(\d+)',txt)
    prof=m.group(1) if m else '?'
    dur=re.search(r'duration_s=(\d+)',txt)
    t_lbt=None; first=None; n=0; last=None; lastf=None; t_end=None
    for l in open(rx,encoding='utf-8-sig',errors='replace'):
        mm=TS.match(l)
        if mm: last=ts(mm.group(1))
        if ('LBT_ENABLE' in l or 'REG_PROFILE' in l) and t_lbt is None and last: t_lbt=last
        if 'frag_arrival' in l or 'published frame_id' in l and 'frag_arrival' not in l:
            pass
        if 'frag_arrival' in l:
            n+=1
            if first is None: first=last
            lastf=last
        if 'shutting down' in l: t_end=last
    if t_lbt is None: continue
    rows.append((os.path.relpath(d,ROOT),prof,dur.group(1) if dur else '?',n,None if first is None else round(first-t_lbt,1),None if t_end is None else round(t_end-t_lbt,1)))
rows.sort()
for r in rows:
    if r[1]=='1': print(r)
