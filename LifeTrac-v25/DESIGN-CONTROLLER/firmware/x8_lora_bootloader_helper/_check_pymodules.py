import sys
print(sys.version)
print("path:", sys.path)
mods = ['fcntl','select','termios','tty','serial','ctypes','struct','time','array']
for m in mods:
    try:
        __import__(m); print(f"  {m} OK")
    except Exception as e:
        print(f"  {m} MISSING ({e})")
import os
for p in sys.path:
    if p and os.path.isdir(p):
        try:
            entries = sorted(os.listdir(p))[:30]
            print(f"-- {p}: {entries}")
        except Exception:
            pass
