#!/usr/bin/env python3
"""Restore docker-compose.yml from tarball, then apply two targeted patches:
   1. Drop mosquitto's host port mapping (avoid 1883 conflict with foundries' mosq).
   2. Escape $LIFETRAC_LORA_DEVICE / $LIFETRAC_MQTT_HOST in the shell command
      so docker compose does not interpolate them away.
"""
import os, subprocess, pathlib, sys
target = pathlib.Path("/opt/lifetrac/DESIGN-CONTROLLER/docker-compose.yml")
# Re-extract just this one file from the tarball.
subprocess.check_call(
    ["tar", "-xzf", "/tmp/lifetrac.tgz", "-C", "/opt/lifetrac",
     "DESIGN-CONTROLLER/docker-compose.yml"])
txt = target.read_text()
# Patch 1: drop the two-line ports block under mosquitto.
needle = (
    "    ports:\n"
    '      - "127.0.0.1:1883:1883"\n'
)
if needle in txt:
    txt = txt.replace(needle, "")
    print("patch1 applied")
else:
    print("patch1: needle not found")
# Patch 2: escape vars in lora_bridge command.
needle2 = '\\"$LIFETRAC_LORA_DEVICE\\" --mqtt \\"$LIFETRAC_MQTT_HOST\\"'
repl2   = '\\"$$LIFETRAC_LORA_DEVICE\\" --mqtt \\"$$LIFETRAC_MQTT_HOST\\"'
if needle2 in txt:
    txt = txt.replace(needle2, repl2)
    print("patch2 applied")
else:
    print("patch2: needle not found")
target.write_text(txt)
print("== verify ==")
print(subprocess.check_output(["grep", "-n", "ports:\\|command:\\|LIFETRAC_LORA_DEVICE\\|LIFETRAC_MQTT_HOST", str(target)]).decode())
