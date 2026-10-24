import math
def wilson(x,n,z):
    p=x/n; d=1+z*z/n; c=p+z*z/(2*n); h=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))
    return (c-h)/d,(c+h)/d
def newc(x1,n1,x2,n2,z):
    p1,p2=x1/n1,x2/n2; l1,u1=wilson(x1,n1,z); l2,u2=wilson(x2,n2,z)
    d=p1-p2
    return d, d-math.sqrt((p1-l1)**2+(u2-p2)**2), d+math.sqrt((u1-p1)**2+(p2-l2)**2)
z=1.6448536269514722
for lab,(v,nv,m,nm) in [('all6',(383,384,542,543)),('excl leg6',(379,379,542,543)),('legs 2+4 only + mono all',(379,379,542,543))]:
    d,lo,hi=newc(v,nv,m,nm,z)
    print(lab,'diff %.3f pp, one-sided 95%% LB %.3f pp, UB %.3f'%(d*100,lo*100,hi*100))
# Fisher exact two-sided
from math import comb
def fisher(a,b,c,d):
    n1=a+b; n2=c+d; k=a+c; N=n1+n2
    def pr(x): return comb(n1,x)*comb(n2,k-x)/comb(N,k)
    p0=pr(a); lo=max(0,k-n2); hi=min(n1,k)
    return sum(pr(x) for x in range(lo,hi+1) if pr(x)<=p0*(1+1e-9))
print('fisher two-sided all6', fisher(383,1,542,1))
# What would happen if leg 6 had been re-flown with ~190 probes at 100%
d,lo,hi=newc(383-4+190,384-5+190,542,543,z); print('if leg6 replaced by 190/190: diff %.3f LB %.3f'%(d*100,lo*100))
# what if the dead leg's absent probes were counted as undelivered (e.g. 601 frames -> ~190 expected probes)
d,lo,hi=newc(383,384+185,542,543,z); print('if 185 missing probes counted undelivered: diff %.2f LB %.2f'%(d*100,lo*100))
