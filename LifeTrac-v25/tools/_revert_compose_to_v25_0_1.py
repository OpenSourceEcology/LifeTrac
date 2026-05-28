#!/usr/bin/env python3
"""Revert base & tractor compose files back to v25.0.1 baseline state
(REG_PROFILE=0, BENCH_ONLY_FIXED_915). Removes the FHSS-related env
vars I added during the 2026-05-27 FHSS bench-default attempt.

Idempotent. Prints OK / ALREADY_REVERTED / NO_MATCH.
"""
import sys, re

def revert_base():
    p = "/var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER/docker-compose.video-test.yml"
    with open(p, "r", encoding="utf-8") as fh:
        s = fh.read()
    # Replace the 3-line FHSS block with the original single line.
    # Account for either the active or commented FORCE_FRF form.
    pattern = re.compile(
        r'      LIFETRAC_REG_PROFILE: "1"\n'
        r'      LIFETRAC_FHSS_WIDE_MASK: "1"\n'
        r'      (#\s*)?LIFETRAC_FORCE_FRF_HZ: "915000000"(  # disabled.*)?'
    )
    target = '      LIFETRAC_REG_PROFILE: "0"'
    if 'LIFETRAC_REG_PROFILE: "0"' in s and 'LIFETRAC_FHSS_WIDE_MASK' not in s:
        return "ALREADY_REVERTED"
    new = pattern.sub(target, s, count=1)
    if new == s:
        return "NO_MATCH"
    with open(p, "w", encoding="utf-8") as fh:
        fh.write(new)
    return "OK"


def revert_tractor():
    p = "/opt/lifetrac/video-test/docker-compose.yml"
    with open(p, "r", encoding="utf-8") as fh:
        s = fh.read()
    pattern = re.compile(
        r'      - LIFETRAC_REG_PROFILE=1\n'
        r'      - LIFETRAC_FHSS_WIDE_MASK=1\n'
        r'      - LIFETRAC_FORCE_FRF_HZ=915000000'
    )
    target = '      - LIFETRAC_REG_PROFILE=0'
    if '      - LIFETRAC_REG_PROFILE=0' in s and 'LIFETRAC_FHSS_WIDE_MASK' not in s:
        return "ALREADY_REVERTED"
    new = pattern.sub(target, s, count=1)
    if new == s:
        return "NO_MATCH"
    with open(p, "w", encoding="utf-8") as fh:
        fh.write(new)
    return "OK"


if __name__ == "__main__":
    role = sys.argv[1] if len(sys.argv) > 1 else "both"
    if role in ("base", "both"):
        print("base:", revert_base())
    if role in ("tractor", "both"):
        print("tractor:", revert_tractor())
