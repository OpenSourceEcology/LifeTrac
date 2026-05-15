#!/bin/sh
echo "=== getent passwd fio ==="
getent passwd fio
echo "=== getent group video ==="
getent group video
echo "=== usermod path ==="
command -v usermod
ls -l "$(command -v usermod)" 2>&1
echo "=== gpasswd path ==="
command -v gpasswd
echo "=== try sudo usermod ==="
echo fio | sudo -S -p '' usermod -aG video fio 2>&1
echo "rc_usermod=$?"
echo "=== id fio ==="
id fio
echo "=== getfacl /dev/video1 ==="
getfacl /dev/video1 2>&1
echo "=== try sudo gpasswd ==="
echo fio | sudo -S -p '' gpasswd -a fio video 2>&1
echo "rc_gpasswd=$?"
echo "=== id fio after ==="
id fio
