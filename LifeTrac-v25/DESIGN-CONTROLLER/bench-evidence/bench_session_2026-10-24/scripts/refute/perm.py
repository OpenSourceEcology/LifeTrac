M=0xFFFFFFFF
def fnv(buf):
    h=0x811C9DC5
    for b in buf:
        h^=b; h=(h*0x01000193)&M
    return h
def seed(e,farm=0,node=0):
    buf=[(farm>>(8*i))&255 for i in range(8)]+[(node>>(8*i))&255 for i in range(8)]+[(e>>(8*i))&255 for i in range(4)]
    return fnv(buf)
def perm(s):
    out=list(range(50)); r=s if s else 0x9E3779B9
    for i in range(49,0,-1):
        x=r; x^=(x<<13)&M; x^=x>>17; x^=(x<<5)&M; r=x
        j=r%(i+1); out[i],out[j]=out[j],out[i]
    return out
def ch(e,h): return perm(seed(e))[h]
if __name__=='__main__':
    for e,h in [(0,0),(13,18),(14,41),(17,22),(18,3),(24,43),(13,17),(14,40),(18,2),(24,43)]:
        print((e,h),ch(e,h))
