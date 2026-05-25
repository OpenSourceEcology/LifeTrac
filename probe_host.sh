#!/bin/sh
echo "=== host python ==="
python3 --version
python3 -c "
mods = ['logging','dataclasses','json','hashlib','queue','select','fcntl','termios','array','socket','struct','threading','collections','enum','typing','pathlib','subprocess','os','sys','time','re','io','signal','argparse','traceback']
import importlib
present, missing = [], []
for m in mods:
    try:
        importlib.import_module(m); present.append(m)
    except Exception as e:
        missing.append((m, str(e).splitlines()[0]))
print('PRESENT', len(present)); print('  ' + ', '.join(present))
print('MISSING', len(missing))
for m,e in missing: print('  ', m, '->', e)
try:
    import paho.mqtt.client; print('PAHO OK')
except Exception as e: print('PAHO MISSING:', e)
"
echo "=== other python interpreters ==="
ls -la /usr/bin/python* /usr/local/bin/python* 2>/dev/null
echo "=== docker python sites available outside container ==="
ls -la /var/sota 2>/dev/null | head -5
