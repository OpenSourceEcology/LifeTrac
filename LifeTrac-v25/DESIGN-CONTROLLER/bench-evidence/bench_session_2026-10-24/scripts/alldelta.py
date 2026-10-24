import ast,sys,os
d=sys.argv[1]
legs=['2c_f1','2b_f1','2c_f2','2b_f2','2c_f3','2b_f3']
def load(p):
    for line in open(p,encoding='utf-8',errors='replace'):
        if line.startswith('ALL:'):
            return ast.literal_eval(line[4:].strip())
    return {}
for side in ['base','tractor']:
    print('#####',side)
    rows={}
    keys=set()
    for l in legs:
        pre=load(os.path.join(d,f'leg{l}_pre_{side}.txt')); post=load(os.path.join(d,f'leg{l}_post_{side}.txt'))
        rows[l]=(pre,post); keys|=set(pre)|set(post)
    print('%-28s'%'key'+''.join('%22s'%l for l in legs))
    for k in sorted(keys):
        s='%-28s'%k
        for l in legs:
            pre,post=rows[l]
            a=pre.get(k,0); b=post.get(k,0)
            s+='%22s'%(f'{a}->{b}')
        print(s)
