#!/bin/sh
set +e
echo '== docker version =='
which docker
docker --version 2>&1
echo '== docker info (short) =='
docker info 2>&1 | grep -E 'Server Version|Architecture|OSType|Storage Driver|Cgroup|Operating System' | head -10
echo '== fio in docker group =='
id fio | tr ',' '\n' | grep docker
echo '== docker permissions test (as fio) =='
docker ps 2>&1 | head -5
echo '== docker compose =='
docker compose version 2>&1 | head -2
docker-compose --version 2>&1 | head -2
echo '== net =='
ping -c 2 -W 2 8.8.8.8 2>&1 | tail -3
ping -c 2 -W 2 registry-1.docker.io 2>&1 | tail -3
echo '== existing images =='
docker images 2>&1 | head -20
echo '== existing containers =='
docker ps -a 2>&1 | head -20
echo '== opkg =='
which opkg
echo '== /dev/ttymxc3 ownership =='
ls -la /dev/ttymxc3
echo '== DONE =='
