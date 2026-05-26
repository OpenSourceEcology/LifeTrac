#!/bin/sh
set -e
PW='fio'

echo "$PW" | sudo -S -p '' mkdir -p /opt/lifetrac
echo "$PW" | sudo -S -p '' chown -R fio:fio /opt/lifetrac
cd /opt/lifetrac
# wipe any prior partial extract
rm -rf DESIGN-CONTROLLER
tar -xzf /tmp/lifetrac.tgz
ls -la DESIGN-CONTROLLER | head -20

cd DESIGN-CONTROLLER

# .env
cat > .env <<EOF
LIFETRAC_PIN=1234
LIFETRAC_LORA_DEVICE=/dev/ttymxc3
LIFETRAC_TRUSTED_PROXIES=
EOF

# secrets dir
mkdir -p secrets
# PIN secret (matches LIFETRAC_PIN)
printf '1234' > secrets/lifetrac_pin
# random 16-byte fleet key (bench only; production must be provisioned)
if [ ! -s secrets/lifetrac_fleet_key ]; then
  python3 -c "import os,sys; sys.stdout.buffer.write(os.urandom(16))" > secrets/lifetrac_fleet_key
fi
chmod 600 secrets/*
ls -la secrets

# Compose file expects ./base_station/mosquitto.conf — already in tarball.
# Override device mount: compose already reads LIFETRAC_LORA_DEVICE from .env.

# Fix the compose file: it maps to /dev/ttyACM0 inside the container as a path,
# which is fine, but lora_bridge command uses LIFETRAC_LORA_DEVICE=/dev/ttyACM0
# inside the container. Confirm that override matches device map.
grep -A2 'lora_bridge:' docker-compose.yml | head -20 || true
grep 'devices:' -A1 docker-compose.yml | head -10

echo "== DONE =="
