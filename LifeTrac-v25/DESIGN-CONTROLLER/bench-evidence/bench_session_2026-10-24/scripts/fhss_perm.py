# Re-implementation of sx1276_fhss.c compute_seed/compute_permutation (farm=0,node=0)
M=0xFFFFFFFF
def fnv(b):
    h=0x811C9DC5
    for x in b:
        h^=x; h=(h*0x01000193)&M
    return h
def seed(epoch,farm=0,node=0):
    b=[(farm>>(8*i))&0xff for i in range(8)]+[(node>>(8*i))&0xff for i in range(8)]+[(epoch>>(8*i))&0xff for i in range(4)]
    return fnv(bytes(b))
def perm(epoch):
    out=list(range(50)); rng=seed(epoch) or 0x9E3779B9
    for i in range(49,0,-1):
        x=rng; x^=(x<<13)&M; x^=x>>17; x^=(x<<5)&M; rng=x&M
        j=rng%(i+1); out[i],out[j]=out[j],out[i]
    return out
def ch(epoch,hop): return perm(epoch)[hop]
if __name__=='__main__':
    # validation against RFCO_PERTX lines
    for e,h,c in [(0,0,3),(13,18,44),(14,41,6),(17,22,6),(18,3,18),(24,43,18),(7,3,30),(2,49,49),(4,34,23),(1,3,11)]:
        print('check',e,h,'->',ch(e,h),'expect',c)
    print('received frames (epoch,hop)->channel')
    for e,h in [(13,17),(14,40),(17,22),(18,2),(24,43)]:
        print(e,h,ch(e,h))
