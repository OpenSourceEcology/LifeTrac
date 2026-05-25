#!/bin/sh
python3 -c "import paho.mqtt.client as c; print('PAHO:', c.__file__)"
echo "---site-packages---"
python3 -c "import site; [print(p) for p in site.getsitepackages()]"
echo "---pip list---"
pip3 list 2>/dev/null | grep -iE "paho|mqtt" || echo "no pip"
echo "---bind mounts---"
mount | grep -E "site-packages|paho" | head -5
echo "---is paho in image or volume?---"
ls -la $(python3 -c "import paho; print(paho.__path__[0])" 2>/dev/null) 2>&1 | head -3
