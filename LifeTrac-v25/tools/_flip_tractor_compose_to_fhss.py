#!/usr/bin/env python3
"""Edit tractor /opt/lifetrac/video-test/docker-compose.yml to flip the
image_tx (lora/video) service into FCC FHSS profile=1 + wide mask + FRF
pin (so v25.0.6.5 bench-default activation matches base). Mirrors
_flip_base_compose_to_fhss.py. Idempotent.

Tractor compose uses list-style env (- KEY=val), distinct from base
(dict style). See _revert_compose_to_v25_0_1.py for inverse.
"""
import sys

PATH = "/opt/lifetrac/video-test/docker-compose.yml"

OLD = "      - LIFETRAC_REG_PROFILE=0"
NEW = (
    "      - LIFETRAC_REG_PROFILE=1\n"
    "      - LIFETRAC_FHSS_WIDE_MASK=1\n"
    "      - LIFETRAC_FORCE_FRF_HZ=915000000"
)

with open(PATH, "r", encoding="utf-8") as fh:
    s = fh.read()

if "LIFETRAC_FHSS_WIDE_MASK" in s and "LIFETRAC_REG_PROFILE=1" in s:
    print("ALREADY_APPLIED")
    sys.exit(0)

if OLD not in s:
    print("NO_MATCH: expected line not found:", repr(OLD))
    sys.exit(1)

s2 = s.replace(OLD, NEW, 1)
with open(PATH, "w", encoding="utf-8") as fh:
    fh.write(s2)
print("OK")
