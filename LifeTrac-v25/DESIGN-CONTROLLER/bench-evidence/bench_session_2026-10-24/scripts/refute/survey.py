import os,re,glob,datetime as dt
B='C:/Users/dorkm/Documents/GitHub/LifeTrac-bench-main/LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence'
def ts(s):
    return dt.datetime.strptime(s,'%Y-%m-%d %H:%M:%S,%f').timestamp()
rows=[]
for p in sorted(glob.glob(B+'/**/params.txt',recursive=True)):
    d=os.path.dirname(p)
    try: par=dict(l.strip().split('=',1) for l in open(p,encoding='utf-8-sig',errors='replace') if '=' in l)
    except Exception: continue
    if par.get('reg_profile')!='1': continue
    rxl=os.path.join(d,'rx_daemon.log')
    if not os.path.exists(rxl): continue
    lbt=None; first=None; nfa=0; stop=None; fps=par.get('synth_fps'); dur=par.get('duration_s')
    for l in open(rxl,encoding='utf-8-sig',errors='replace'):
        if lbt is None and 'LBT_ENABLE' in l:
            try: lbt=ts(l[:23])
            except: pass
        if 'frag_arrival' in l:
            nfa+=1
            if first is None:
                try: first=ts(l[:23])
                except: pass
        if 'signal received' in l:
            try: stop=ts(l[:23])
            except: pass
    if nfa==0: continue
    rows.append((os.path.relpath(d,B),fps,dur,round(first-lbt,1) if lbt and first else None, round(stop-lbt,1) if stop and lbt else None,nfa))
for r in rows: print(r)
print(len(rows))
