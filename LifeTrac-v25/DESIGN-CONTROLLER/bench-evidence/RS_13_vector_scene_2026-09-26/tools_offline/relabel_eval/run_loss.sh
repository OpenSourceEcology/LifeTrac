#!/bin/sh
cd "$(dirname "$0")"
export PYTHONIOENCODING=utf-8
py -3 rl_loss.py A "lab40 (today),pix50,cov40,val40" "0.0,0.03,0.08" 1 > loss_A.log 2>&1
echo finished
cat loss_A.log
