import numpy as np, math
rng = np.random.default_rng(13)
TCRIT = {1:12.706,2:4.303,3:3.182,4:2.776,5:2.571,6:2.447,7:2.365,8:2.306,9:2.262,10:2.228,11:2.201}
def sim(k, sigma, rho, r, p0=0.03, n=600, sims=20000):
    s = rng.normal(0, sigma*math.sqrt(rho), (sims, k))
    ec = rng.normal(0, sigma*math.sqrt(1-rho), (sims, k))
    ev = rng.normal(0, sigma*math.sqrt(1-rho), (sims, k))
    pc = np.clip(p0*np.exp(s+ec-sigma**2/2), 0, 1)
    pv = np.clip(r*p0*np.exp(s+ev-sigma**2/2), 0, 1)
    xc = rng.binomial(n, pc); xv = rng.binomial(n, pv)
    d = np.log((xv+0.5)/(n+0.5)) - np.log((xc+0.5)/(n+0.5))
    # paired t on log ratio
    if k >= 2:
        m = d.mean(1); sd = d.std(1, ddof=1)
        t = m/(sd/math.sqrt(k))
        pt = np.mean(np.abs(t) > TCRIT[k-1])
    else:
        pt = float('nan')
    # pooled CMH (assumes binomial, no leg effect): chi2 1 df
    N = 2*n
    a = xv; c = xc; m1 = a + c
    E = (m1*n/N).sum(1)
    V = (m1*(N-m1)*n*n/(N*N*(N-1))).sum(1)
    chi = (np.abs(a.sum(1)-E)-0.5)**2/np.where(V>0, V, np.nan)
    pc_ = np.nanmean(chi > 3.841)
    return pt, pc_
print("k pairs of interleaved 600-frame legs (VECTOR, control), p0 = 3 %, leg-level lognormal sigma, within-session correlation rho")
print("cols: paired-t power @x2 | paired-t type-I @x1 | pooled-CMH power @x2 | pooled-CMH type-I @x1")
for sigma in (0.0, 0.5, 1.0):
    for rho in ((0.0,) if sigma == 0 else (0.0, 0.5, 0.8)):
        for k in (1, 2, 3, 4, 6, 8):
            pt2, pc2 = sim(k, sigma, rho, 2.0)
            pt1, pc1 = sim(k, sigma, rho, 1.0)
            print(f"sigma {sigma:.1f} rho {rho:.1f} k={k}:  t {pt2:.2f} | t-I {pt1:.2f} | CMH {pc2:.2f} | CMH-I {pc1:.2f}")
