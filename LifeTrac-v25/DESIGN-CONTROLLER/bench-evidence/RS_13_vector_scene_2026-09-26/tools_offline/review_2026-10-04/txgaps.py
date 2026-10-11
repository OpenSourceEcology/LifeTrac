import re, sys, datetime
pat = re.compile(r'^(\S+ \S+),(\d+) INFO image_tx_daemon: frame seq=(\d+) done')
for path in sys.argv[1:]:
    ts = []
    for line in open(path, encoding='utf-8', errors='replace'):
        m = pat.match(line.lstrip('\ufeff'))
        if m:
            t = datetime.datetime.strptime(m.group(1), '%Y-%m-%d %H:%M:%S').timestamp() + int(m.group(2))/1000
            ts.append((t, int(m.group(3))))
    g = [(ts[i][0]-ts[i-1][0], ts[i][1]) for i in range(1, len(ts))]
    gs = sorted(x[0] for x in g)
    print(path, 'tx frames', len(ts))
    if gs:
        print('  gap p50 %.3f p95 %.3f max %.3f; >=1.0 s: %d; >1.5: %d; >2.0: %d' % (gs[len(gs)//2], gs[int(len(gs)*.95)], gs[-1], sum(x>=1.0 for x in gs), sum(x>1.5 for x in gs), sum(x>2.0 for x in gs)))
        print('  gaps >= 0.95 s (gap, tx seq):', [(round(a,3), b) for a, b in g if a >= 0.95][:20])
        # longest run of consecutive gaps < 1.0 s
        run = best = 1
        for a, b in g:
            run = run + 1 if a < 1.0 else 1
            best = max(best, run)
        print('  longest chain of TX_DONE gaps < 1.0 s (proxy for streak):', best)
