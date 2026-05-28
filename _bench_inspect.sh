#!/bin/bash
grep -nE 'services:|^  [a-z_]+:|build:|image:' /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER/docker-compose.yml
echo "---OVERRIDE---"
grep -nE 'services:|^  [a-z_]+:|build:|image:' /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER/docker-compose.video-test.yml
