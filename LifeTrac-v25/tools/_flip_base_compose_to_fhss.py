#!/usr/bin/env python3
"""Edit base station docker-compose.video-test.yml to flip the image_rx
service to FCC FHSS profile=1 with wide mask + FRF pin (so the daemon's
configure_regulatory_profile_if_needed() activates profile=1 cleanly and
both peers land on the same carrier despite the 2026-05-25 channel-
mismatch bug). Idempotent: re-running is a no-op once applied.
"""
import sys

PATH = "/var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER/docker-compose.video-test.yml"

OLD = '      LIFETRAC_REG_PROFILE: "0"'
NEW = (
    '      LIFETRAC_REG_PROFILE: "1"\n'
    '      LIFETRAC_FHSS_WIDE_MASK: "1"\n'
    '      LIFETRAC_FORCE_FRF_HZ: "915000000"'
)

with open(PATH, "r", encoding="utf-8") as fh:
    s = fh.read()

if 'LIFETRAC_FHSS_WIDE_MASK' in s and 'LIFETRAC_REG_PROFILE: "1"' in s:
    print("ALREADY_APPLIED")
    sys.exit(0)

if OLD not in s:
    print("NO_MATCH: expected line not found:", repr(OLD))
    sys.exit(1)

s2 = s.replace(OLD, NEW, 1)
with open(PATH, "w", encoding="utf-8") as fh:
    fh.write(s2)
print("OK")
