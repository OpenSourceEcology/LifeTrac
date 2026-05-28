#!/usr/bin/env python3
"""Toggle LIFETRAC_FORCE_FRF_HZ on the base image_rx service.

Usage: python3 _flip_base_frf_pin.py {on|off}

  off: comment out the env var (lets firmware RX scan policy drive FRF
       across the FHSS mask). Use to test real wide-mask FHSS RX.
  on:  uncomment / restore. Pins RX to 915 MHz (single-channel mode).
"""
import sys

PATH = "/var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER/docker-compose.video-test.yml"
ACTIVE = '      LIFETRAC_FORCE_FRF_HZ: "915000000"'
COMMENTED = '      # LIFETRAC_FORCE_FRF_HZ: "915000000"  # disabled — letting RX scan policy drive FRF under wide-mask FHSS'

if len(sys.argv) != 2 or sys.argv[1] not in ("on", "off"):
    print("usage: _flip_base_frf_pin.py {on|off}")
    sys.exit(2)
mode = sys.argv[1]

with open(PATH, "r", encoding="utf-8") as fh:
    s = fh.read()

if mode == "off":
    if ACTIVE not in s:
        if COMMENTED in s:
            print("ALREADY_OFF")
            sys.exit(0)
        print("NO_MATCH (active line missing)")
        sys.exit(1)
    s2 = s.replace(ACTIVE, COMMENTED, 1)
else:  # on
    if COMMENTED not in s:
        if ACTIVE in s:
            print("ALREADY_ON")
            sys.exit(0)
        print("NO_MATCH (commented line missing)")
        sys.exit(1)
    s2 = s.replace(COMMENTED, ACTIVE, 1)

with open(PATH, "w", encoding="utf-8") as fh:
    fh.write(s2)
print(f"OK: set to {mode}")
