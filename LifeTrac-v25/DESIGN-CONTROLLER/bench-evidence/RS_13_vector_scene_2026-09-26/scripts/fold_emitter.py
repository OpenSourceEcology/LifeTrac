"""RS-13.1 A19: do a leg's losses (seq gaps) and hot CRC dumps fold on the RS-11.6
7.07-7.09 s emitter grid? Reads each archive's rx_daemon.log only (single base clock).

Usage: py -3 fold_emitter.py radio_monitor_<archive> [...]   (archives under bench-evidence/)"""
import re, sys, math, random, pathlib
ROOT = pathlib.Path(__file__).resolve().parents[2]          # bench-evidence/
TS = re.compile(r"^(\d{4}-\d{2}-\d{2}) (\d{2}):(\d{2}):(\d{2}),(\d{3})")
FA = re.compile(r"frag_arrival: seq=(\d+) idx=0")
CD = re.compile(r"crc_dump:.*snr=(-?[\d.]+) rssi=(-?\d+)")
def t_of(line):
    m = TS.match(line)
    if not m: return None
    return int(m.group(2))*3600+int(m.group(3))*60+int(m.group(4))+int(m.group(5))/1000
def rayleigh(ts, P):
    if len(ts) < 2: return 0.0, 1.0
    c = sum(math.cos(2*math.pi*t/P) for t in ts); s = sum(math.sin(2*math.pi*t/P) for t in ts)
    n = len(ts); R = math.hypot(c, s)/n; z = n*R*R
    p = math.exp(-z)*(1+(2*z-z*z)/(4*n)-(24*z-132*z*z+76*z**3-9*z**4)/(288*n*n))
    return R, max(min(p,1.0),0.0)
def leg(arch):
    arr = []; hot = []
    for ln in (ROOT/arch/"rx_daemon.log").read_text(encoding="utf-8", errors="replace").splitlines():
        t = t_of(ln)
        if t is None: continue
        m = FA.search(ln)
        if m: arr.append((t, int(m.group(1)))); continue
        m = CD.search(ln)
        if m and int(m.group(2)) > -60 and float(m.group(1)) < -5: hot.append(t)
    lost = []
    for (t0, s0), (t1, s1) in zip(arr, arr[1:]):
        d = (s1 - s0) % 65536
        if 1 < d < 50:
            for k in range(1, d):
                lost.append(t0 + (t1-t0)*k/d)
    return arr, lost, hot
archs = sys.argv[1:]
for a in archs:
    arr, lost, hot = leg(a)
    ev = sorted(lost + hot)
    print(f"{a}: frames {len(arr)} lost {len(lost)} hot_crc {len(hot)}")
    best = max(((rayleigh(lost, P/10000.0)[0], P/10000.0) for P in range(68000, 72001, 5)), default=(0,0))
    for P in (7.0733, 7.0842, 7.0853, 7.02):
        R, p = rayleigh(lost, P); Rh, ph = rayleigh(hot, P)
        print(f"   P={P:.4f}  lost R={R:.3f} p={p:.3g}   hot R={Rh:.3f} p={ph:.3g}")
    print(f"   scan 6.8-7.2 s on lost: best R={best[0]:.3f} at P={best[1]:.4f}")
    # null: random loss times on the same frame grid
    if arr and lost:
        frame_t = [t for t, _ in arr]
        cnt = 0; N = 2000
        Robs = max(rayleigh(lost, P/10000.0)[0] for P in range(70700, 70901, 5))
        for _ in range(N):
            fake = random.sample(frame_t, min(len(lost), len(frame_t)))
            Rn = max(rayleigh(fake, P/10000.0)[0] for P in range(70700, 70901, 5))
            if Rn >= Robs: cnt += 1
        print(f"   max R over 7.070-7.090 s: {Robs:.3f}; random-frame null P(R>=obs) = {cnt/N:.3f}")
