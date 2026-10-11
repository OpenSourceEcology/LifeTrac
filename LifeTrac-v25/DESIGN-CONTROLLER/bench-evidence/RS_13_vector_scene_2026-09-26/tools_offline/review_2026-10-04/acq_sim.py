# Monte-Carlo of FHSS cold acquisition: base scans channels 0..49 linearly, 500 ms per channel
# (sx1276_rx_scan_policy.h SX1276_RX_SCAN_DWELL_MS, sx1276_rx_scan_walker.h); tractor hops a fresh
# pseudo-random 50-channel permutation per 10 s epoch on 200 ms slots (sx1276_fhss_clock.h) and keys
# each frame 12 ms into a slot (SLOT_TX_HEADSTART_MS). Model only -- not the firmware.
import numpy as np
rng = np.random.default_rng(7)
SLOT = 0.2; DWELL = 0.5; NCH = 50
def tx_slots(pattern, horizon_slots, offset):
    # pattern: list of slot increments cycling, e.g. 2 fps = [2,3] (0.4/0.6 s), 1 fps = [5]
    s = []; k = offset; i = 0
    while k < horizon_slots:
        s.append(k); k += pattern[i % len(pattern)]; i += 1
    return np.array(s)
def one(pattern, toa, rule, horizon_s=300.0):
    hs = int(horizon_s/SLOT)
    n_ep = hs//NCH + 2
    perms = np.array([rng.permutation(NCH) for _ in range(n_ep)])
    off = rng.integers(0, 5)
    slots = tx_slots(pattern, hs, off)
    scan_phase = rng.uniform(0, DWELL)          # scanner dwell boundary vs slot grid
    scan_ch0 = rng.integers(0, NCH)             # base scan starts at an arbitrary point of the walk (field case)
    for k in slots:
        ch = perms[k//NCH][k % NCH]
        t0 = k*SLOT + 0.012; t1 = t0 + toa
        d = int((t0 + scan_phase)//DWELL)       # dwell index at packet start
        dch = (scan_ch0 + d) % NCH
        if dch != ch: continue
        dwell_end = (d+1)*DWELL - scan_phase
        if rule == 'start' or t1 <= dwell_end:
            return t1
    return np.inf
for name, pat, toa in [("VECTOR 2 fps, 1 frame/slot (0.4/0.6 s)", [2,3], 0.169), ("VECTOR 1 fps", [5], 0.169), ("dense: every slot", [1], 0.169)]:
    for rule in ('start', 'contained'):
        t = np.array([one(pat, toa, rule) for _ in range(4000)])
        f = t[np.isfinite(t)]
        print(f"{name:42s} rule={rule:9s} median {np.median(t):5.1f} s  mean {f.mean():5.1f}  p90 {np.quantile(t,0.9):5.1f}  P(>30 s) {np.mean(t>30):.2f}  P(>45 s) {np.mean(t>45):.2f}  P(>60 s) {np.mean(t>60):.2f}")
