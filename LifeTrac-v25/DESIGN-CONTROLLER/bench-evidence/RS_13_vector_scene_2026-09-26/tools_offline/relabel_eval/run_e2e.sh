#!/bin/sh
cd "$(dirname "$0")"
RULES="lab40 (today),val40,l1v40,lab50,lab60,lab40&val>=10,lab40&val>=15,lab40&val>=20,mc_lab40,cov40,pix40,pix50,pix60,pixmc50"
export PYTHONIOENCODING=utf-8
py -3 rl_e2e.py loss0 0.0 "$RULES" > e2e_loss0.log 2>&1
py -3 rl_e2e.py loss8 0.08 "$RULES" > e2e_loss8.log 2>&1
echo finished
tail -2 e2e_loss0.log e2e_loss8.log
