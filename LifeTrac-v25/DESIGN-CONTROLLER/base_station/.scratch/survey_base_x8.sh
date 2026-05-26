#!/bin/sh
set +e
echo '== uname/os =='
uname -a
head -5 /etc/os-release

echo '== python =='
for p in python python3 python3.9 python3.10 python3.11 python3.12; do
  if which "$p" >/dev/null 2>&1; then
    echo -n "$p: "
    "$p" --version 2>&1
  fi
done

echo '== pip =='
for p in pip pip3; do which "$p" 2>/dev/null; done
python3 -m pip --version 2>&1 | head -1

echo '== stdlib check =='
python3 -c 'import fcntl,mmap,select,socket,struct,ssl,json,sqlite3,asyncio; print("stdlib ok")' 2>&1

echo '== installed pkgs of interest =='
python3 - <<'PYEOF'
import importlib
for m in ["fastapi","starlette","uvicorn","pydantic","paho.mqtt","amqtt","cryptography","serial","httpx","websockets","jinja2"]:
    try:
        mod = importlib.import_module(m)
        print(m, "ok", getattr(mod, "__version__", "?"))
    except Exception as e:
        print(m, "MISSING", type(e).__name__)
PYEOF

echo '== disk =='
df -h / /home /tmp 2>/dev/null

echo '== tty =='
ls -la /dev/ttymxc* 2>/dev/null
groups fio
stty -F /dev/ttymxc3 2>&1 | head -2

echo '== systemctl =='
which systemctl
systemctl --version 2>&1 | head -1

echo '== ports =='
ss -tlnp 2>/dev/null | head -20 || netstat -tlnp 2>/dev/null | head -20

echo '== existing lifetrac dirs =='
ls -la /home/fio/ 2>/dev/null
find /home/fio -maxdepth 3 -name "*.py" 2>/dev/null | head -20

echo '== DONE =='
