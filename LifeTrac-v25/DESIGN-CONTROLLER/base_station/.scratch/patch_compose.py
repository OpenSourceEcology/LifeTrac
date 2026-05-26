#!/usr/bin/env python3
"""Patch /opt/lifetrac/DESIGN-CONTROLLER/docker-compose.yml: escape $VAR -> $$VAR
for the lora_bridge command so docker-compose does not interpolate them at parse
time."""
import pathlib
p = pathlib.Path("/opt/lifetrac/DESIGN-CONTROLLER/docker-compose.yml")
txt = p.read_text()
new = (txt
       .replace('\\"$LIFETRAC_LORA_DEVICE\\"', '\\"$$LIFETRAC_LORA_DEVICE\\"')
       .replace('\\"$LIFETRAC_MQTT_HOST\\"',   '\\"$$LIFETRAC_MQTT_HOST\\"'))
if new != txt:
    p.write_text(new)
    print("patched")
else:
    print("no change")
