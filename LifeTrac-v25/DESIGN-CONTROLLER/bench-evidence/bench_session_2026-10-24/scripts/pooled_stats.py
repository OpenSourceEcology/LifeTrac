#!/usr/bin/env python3
"""Own implementation: Fisher exact (two-sided, exact rationals) and the
Newcombe hybrid score interval (method 10) from Wilson score intervals.
Does not import rs13_probe_score."""
from fractions import Fraction
from math import comb, sqrt, erf


def norm_ppf(p):
    # bisection on the normal CDF (no statistics module)
    lo, hi = -10.0, 10.0
    for _ in range(200):
        mid = (lo + hi) / 2
        if 0.5 * (1 + erf(mid / sqrt(2))) < p:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def fisher(a, b, c, d):
    """two-sided: sum of P(table) over tables with P <= P(observed), exact rationals."""
    r1, r2, c1 = a + b, c + d, a + c
    n = r1 + r2
    tot = comb(n, c1)
    def P(x):
        return Fraction(comb(r1, x) * comb(r2, c1 - x), tot)
    po = P(a)
    lo, hi = max(0, c1 - r2), min(c1, r1)
    ps = [P(x) for x in range(lo, hi + 1)]
    two = sum(p for p in ps if p <= po)
    left = sum(P(x) for x in range(lo, a + 1))
    right = sum(P(x) for x in range(a, hi + 1))
    return float(two), float(left), float(right), float(min(1, 2 * min(left, right)))


def wilson(x, n, z):
    p = x / n
    den = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = (z / den) * sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return centre - half, centre + half


def newcombe10(x1, n1, x2, n2, z):
    p1, p2 = x1 / n1, x2 / n2
    l1, u1 = wilson(x1, n1, z)
    l2, u2 = wilson(x2, n2, z)
    d = p1 - p2
    return d - sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2), d + sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2)


z95two = norm_ppf(0.975)
z95one = norm_ppf(0.95)
print(f"z(0.975)={z95two:.6f} z(0.95)={z95one:.6f}")
# check against Newcombe 1998 Table II method 10 (95 % two-sided)
for (x1, n1, x2, n2, ref) in [(56, 70, 48, 80, (0.0524, 0.3339)), (9, 10, 3, 10, (0.1705, 0.8090)),
                              (10, 10, 0, 20, (0.6791, 1.0000))]:
    lo, hi = newcombe10(x1, n1, x2, n2, z95two)
    print(f"check {x1}/{n1} - {x2}/{n2}: {lo:.4f} .. {hi:.4f} (paper {ref[0]:.4f} .. {ref[1]:.4f})")

legs = {"2c_f1": ("mono", 161, 161), "2b_f1": ("vec", 191, 191), "2c_f2": ("mono", 179, 179),
        "2b_f2": ("vec", 188, 188), "2c_f3": ("mono", 202, 203), "2b_f3": ("vec", 4, 5)}


def pool(excl=()):
    v = [0, 0]
    m = [0, 0]
    for k, (c, d, s) in legs.items():
        if k in excl:
            continue
        t = v if c == "vec" else m
        t[0] += d
        t[1] += s
    return v, m


for label, excl in (("all six (pre-written rule)", ()), ("leg 6 2b_f3 excluded (information only)", ("2b_f3",))):
    (dv, sv), (dm, sm) = pool(excl)
    two, left, right, dbl = fisher(dv, sv - dv, dm, sm - dm)
    lo1, hi1 = newcombe10(dv, sv, dm, sm, z95one)
    lo2, hi2 = newcombe10(dv, sv, dm, sm, z95two)
    diff = dv / sv - dm / sm
    print(f"\n{label}: VECTOR {dv}/{sv} = {100*dv/sv:.3f} %; mono_g4 {dm}/{sm} = {100*dm/sm:.3f} %; diff {100*diff:+.3f} pp")
    print(f"  Fisher two-sided (sum of P<=Pobs) p = {two:.6f}; one-sided left p = {left:.6f}, right p = {right:.6f}; doubled-min p = {dbl:.6f}")
    print(f"  Newcombe hybrid (method 10): one-sided 95 % lower bound {100*lo1:+.3f} pp (90 % two-sided {100*lo1:+.3f} .. {100*hi1:+.3f}); 95 % two-sided {100*lo2:+.3f} .. {100*hi2:+.3f}")
    print(f"  Wilson VECTOR 90%: {wilson(dv, sv, z95one)}; mono 90%: {wilson(dm, sm, z95one)}")
    print(f"  margin -3 pp: {'NOT WORSE' if lo1 > -0.03 else 'not shown'} (lower bound {'>' if lo1 > -0.03 else '<='} -3 pp)")
