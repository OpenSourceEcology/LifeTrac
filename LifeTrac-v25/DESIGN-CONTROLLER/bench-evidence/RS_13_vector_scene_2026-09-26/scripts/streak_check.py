"""When could the tractor first become FHSS time authority? Per firmware
sx1276_fhss_authority.h: MIN_STREAK=8 consecutive own TX, each STRICTLY < 1000 ms
after the previous. Uses the tractor tx_daemon.log 'frame seq=N done' timestamps
(the daemon logs TX_DONE; tractor clock, consistent within the log)."""
import re, sys, datetime as dt
GAP, MINS = 1000.0, 8
pat = re.compile(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3}) .*frame seq=(\d+) done")
for path in sys.argv[1:]:
    ts, seqs = [], []
    for ln in open(path, encoding="utf-8", errors="replace"):
        m = pat.search(ln)
        if m:
            ts.append(dt.datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S,%f")); seqs.append(int(m.group(2)))
    gaps = [(b - a).total_seconds() * 1000.0 for a, b in zip(ts, ts[1:])]
    streak, first = 1, None
    for i, g in enumerate(gaps, start=1):
        streak = streak + 1 if g < GAP else 1
        if streak >= MINS and first is None:
            first = i
    n_lt = sum(1 for g in gaps if g < GAP)
    print(f"{path.split('/')[-2]}: {len(ts)} TX; gaps < 1000 ms: {n_lt}/{len(gaps)}; gap ms p10/p50/p90 "
          f"{sorted(gaps)[len(gaps)//10]:.0f}/{sorted(gaps)[len(gaps)//2]:.0f}/{sorted(gaps)[9*len(gaps)//10]:.0f}; "
          f"first 8-streak completes at frame #{first + 1 if first is not None else None} (seq {seqs[first] if first is not None else None})")
