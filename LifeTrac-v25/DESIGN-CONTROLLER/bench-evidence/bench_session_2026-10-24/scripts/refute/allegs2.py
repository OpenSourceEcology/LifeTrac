exec(open('allegs.py').read().split("for arch,known in")[0])
for arch in ['radio_monitor_20261024_100249_d28bc2ca','radio_monitor_20261024_103156_d28bc2ca','radio_monitor_20261024_104056_d28bc2ca']:
    tx,rx,probes,lbt=load(arch)
    P,b,starts=fit(tx,probes) if probes and len(probes)>10 else (None,None,None)
    if P is None: continue
    ph=[(p[0]-b)/P-p[1] for p in probes]
    print(arch[14:29],'probe ok-time phase vs RFCO slot: min %.2f max %.2f'%(min(ph),max(ph)))
    slots=[math.floor((s-b)/P+0.5) for s in starts]
    first=rx[0][0]; fi=min(range(len(tx)),key=lambda i:abs(tx[i][0]-first))
    s=slots[fi]; rel=first-lbt
    print(' first frame #%d slot %d (%d,%d) ch %d; rel LBT %.2f -> attempt %d dwell %d walker ch %d'%(fi+1,s,s//50,s%50,ch(s//50,s%50),rel,int(rel//30),int((rel%30)//0.5),int((rel%30)//0.5)%50))
    # predicted early catches at offset 0
    Ta=lbt
    for i,(sl,(t,sq,toa)) in enumerate(zip(slots,tx)):
        st=b+P*sl+0.014; en=st+toa/1e6; rel=st-Ta
        if rel<0: continue
        a=int(rel//30); d=int((rel-30*a)//0.5)
        if a>3: break
        if d<=59 and ch(sl//50,sl%50)==d%50:
            print('   frame #%d slot (%d,%d) ch %d at rel %.3f dwell %d [%.1f-%.1f] ends %.3f %s'%(i+1,sl//50,sl%50,d%50,rel,d,30*a+0.5*d,30*a+0.5*d+0.5,en-Ta,'FITS' if en<=Ta+30*a+0.5*(d+1) else 'overruns'))
            if i>fi: break
