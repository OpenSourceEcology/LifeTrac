import math, random
from functools import lru_cache

def lfact(n, _c={}):
    v = _c.get(n)
    if v is None:
        v = math.lgamma(n + 1); _c[n] = v
    return v

def binom_pmf(n, p):
    return [math.exp(lfact(n) - lfact(k) - lfact(n-k) + (k*math.log(p) if k else 0) + ((n-k)*math.log1p(-p) if n-k else 0)) for k in range(n+1)]

@lru_cache(maxsize=None)
def fisher_reject_set(n1, n2, K, alpha=0.05):
    """two-sided Fisher (sum of P <= P_obs) rejection set for x1 given total K"""
    lo, hi = max(0, K-n2), min(n1, K)
    lp = {x: lfact(K)-lfact(x)-lfact(K-x) + lfact(n1+n2-K)-lfact(n1-x)-lfact(n2-K+x) - (lfact(n1+n2)-lfact(n1)-lfact(n2)) for x in range(lo, hi+1)}
    probs = {x: math.exp(v) for x, v in lp.items()}
    rej = set()
    for x in range(lo, hi+1):
        px = probs[x]
        pval = sum(p for p in probs.values() if p <= px*(1+1e-7))
        if pval <= alpha: rej.add(x)
    return frozenset(rej)

def fisher_p(x1, n1, x2, n2):
    K = x1 + x2
    lo, hi = max(0, K-n2), min(n1, K)
    probs = {x: math.exp(lfact(K)-lfact(x)-lfact(K-x) + lfact(n1+n2-K)-lfact(n1-x)-lfact(n2-K+x) - (lfact(n1+n2)-lfact(n1)-lfact(n2))) for x in range(lo, hi+1)}
    px = probs[x1]
    return sum(p for p in probs.values() if p <= px*(1+1e-7))

def fisher_power(n, p0, ratio, alpha=0.05):
    p1 = p0*ratio
    a = binom_pmf(n, p1); b = binom_pmf(n, p0)
    ka = [k for k in range(n+1) if a[k] > 1e-10]; kb = [k for k in range(n+1) if b[k] > 1e-10]
    pw = 0.0
    for x1 in ka:
        for x2 in kb:
            if x1 in fisher_reject_set(n, n, x1+x2, alpha):
                pw += a[x1]*b[x2]
    return pw

print("== Observed P3 data (seq-gap loss) ==")
for tag, xv, nv, xc, nc in [("round 4 (2 fps) 2a vs 2c", 14, 610, 21, 612), ("round 3 (1 fps) 2a vs 2c", 4, 303, 1, 304), ("round 2 (1 fps) 2a vs 2c", 3, 303, 20, 304)]:
    rr = (xv/nv)/(xc/nc)
    se = math.sqrt(1/xv - 1/nv + 1/xc - 1/nc)
    print(f"{tag}: VECTOR {xv}/{nv}={100*xv/nv:.2f}% control {xc}/{nc}={100*xc/nc:.2f}%  Fisher p={fisher_p(xv,nv,xc,nc):.3f}  RR={rr:.2f} 95% CI {rr*math.exp(-1.96*se):.2f}-{rr*math.exp(1.96*se):.2f}")

# Mantel-Haenszel RR across the three DTS rounds (strata = sessions), Greenland-Robins variance
strata = [(14,610,21,612),(4,303,1,304),(3,303,20,304)]
R = S = 0.0; P = 0.0
for a, n1, c, n0 in strata:
    N = n1+n0
    R += a*n0/N; S += c*n1/N
    P += (n1*n0*(a+c) - a*c*N)/N**2
rr_mh = R/S
var_ln = P/(R*S)
for de in (1, 2, 3):
    se = math.sqrt(var_ln*de)
    print(f"MH pooled RR (3 sessions) = {rr_mh:.2f}, 95% CI {rr_mh*math.exp(-1.96*se):.2f}-{rr_mh*math.exp(1.96*se):.2f}  [variance x{de}]")

# homogeneity of the three control legs
ctrl = [(20,304),(1,304),(21,612)]
X = sum(x for x,_ in ctrl); Nn = sum(n for _,n in ctrl); p = X/Nn
chi = sum((x-n*p)**2/(n*p) + ((n-x)-n*(1-p))**2/(n*(1-p)) for x,n in ctrl)
print(f"control legs homogeneity: chi2={chi:.1f}, df=2, p={math.exp(-chi/2):.2e}  (pooled {100*p:.2f}%)")
vec = [(3,303),(4,303),(14,610)]
X = sum(x for x,_ in vec); Nn = sum(n for _,n in vec); p = X/Nn
chi = sum((x-n*p)**2/(n*p) + ((n-x)-n*(1-p))**2/(n*(1-p)) for x,n in vec)
print(f"VECTOR DTS legs homogeneity: chi2={chi:.1f}, df=2, p={math.exp(-chi/2):.2e}  (pooled {100*p:.2f}%)")

print("\n== Power of a two-sided Fisher exact test (alpha 0.05), binomial frames ==")
for n in (300, 600, 1200, 1800, 2400, 3600):
    row = []
    for r in (1.5, 2.0, 3.0):
        row.append(f"x{r}: {fisher_power(n, 0.03, r):.2f}")
    print(f"n={n:5d}/arm, p0=3%: " + "  ".join(row))
for n in (600, 1200, 2400):
    print(f"n={n:5d}/arm, p0=1%: x2.0: {fisher_power(n, 0.01, 2.0):.2f}   p0=0.4% x2.0: {fisher_power(n, 0.004, 2.0):.2f}")
# smallest n for 80 % at x2 and x1.5, p0=3%
for r in (2.0, 1.5):
    lo, hi = 300, 12000
    while hi - lo > 50:
        mid = (lo+hi)//2
        if fisher_power(mid, 0.03, r) >= 0.8: hi = mid
        else: lo = mid
    print(f"n per arm for 80% power at x{r}, p0=3%: ~{hi} frames (= {hi/600:.1f} legs of 600 frames per arm)")
