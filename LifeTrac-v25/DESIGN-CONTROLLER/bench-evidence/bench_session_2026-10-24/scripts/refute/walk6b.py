exec(open('walk6.py').read().split("for o in")[0])
for off_ms in (0.0,8.0,14.0,20.0):
    row=[]
    for o in range(-60,121,10):
        r=sim(lbt+o/1000,off_ms)
        row.append('%+d:%s'%(o, 'none' if r is None else '#%d@%.1fs'%(r[0],r[2])))
    print('slot_off %4.1f ms |'%off_ms,' '.join(row))
# frame 6 details
i=5
s=slots[i]; print('frame6 slot',s,(s//50,s%50),'ch',ch(s//50,s%50),'toa',tx[i][2], 'start rel LBT %.3f'%(b+P*s+0.014-lbt))
