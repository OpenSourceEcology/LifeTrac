"""Python mirror of radio/sx1276_fhss_chantab.c.

Single source of truth for the 50-channel FCC §15.247(a)(1)(i)
narrowband hopset shared between the L072 firmware and any X8-side
Python tooling (bench scripts, evidence post-processors, orchestrators).

If this file and the C table ever disagree, FCC-EVID-D1 (hop proof) will
fail by construction because the spectrum analyzer expectations and the
firmware reality will not line up. Treat the two files as one unit:
edit them in the same commit.

Golden vectors (idx -> Hz) below match k_golden[] in
fhss_chantab_vectors.c exactly.
"""

from __future__ import annotations

SX1276_FHSS_CHANNEL_COUNT = 50
SX1276_FHSS_CHANNEL_SPACING_HZ = 500_000
SX1276_FHSS_FIRST_CENTER_HZ = 902_750_000
SX1276_FHSS_LAST_CENTER_HZ = (
    SX1276_FHSS_FIRST_CENTER_HZ
    + (SX1276_FHSS_CHANNEL_COUNT - 1) * SX1276_FHSS_CHANNEL_SPACING_HZ
)

SX1276_FHSS_BAND_LOWER_HZ = 902_000_000
SX1276_FHSS_BAND_UPPER_HZ = 928_000_000

# Module-load invariants — match the C _Static_assert set.
assert SX1276_FHSS_CHANNEL_COUNT == 50
assert SX1276_FHSS_CHANNEL_SPACING_HZ >= 25_000
assert SX1276_FHSS_FIRST_CENTER_HZ >= SX1276_FHSS_BAND_LOWER_HZ
assert SX1276_FHSS_LAST_CENTER_HZ <= SX1276_FHSS_BAND_UPPER_HZ


def center_hz(idx: int) -> int:
    """Return the center frequency in Hz for channel index 0..49.

    Returns 0 for out-of-range indices (matches the C ABI: caller must
    treat 0 as fault).
    """
    if not 0 <= idx < SX1276_FHSS_CHANNEL_COUNT:
        return 0
    return SX1276_FHSS_FIRST_CENTER_HZ + idx * SX1276_FHSS_CHANNEL_SPACING_HZ


def count() -> int:
    return SX1276_FHSS_CHANNEL_COUNT


# Golden vectors — must match k_golden[] in fhss_chantab_vectors.c.
GOLDEN = {
    0: 902_750_000,
    1: 903_250_000,
    9: 907_250_000,
    24: 914_750_000,
    25: 915_250_000,
    49: 927_250_000,
}


def _self_test() -> int:
    failures = 0
    if count() != 50:
        print(f"[FAIL] count == 50 (got {count()})")
        failures += 1
    for idx, expected in GOLDEN.items():
        got = center_hz(idx)
        if got != expected:
            print(f"[FAIL] golden idx={idx} expected={expected} got={got}")
            failures += 1
    for i in range(1, SX1276_FHSS_CHANNEL_COUNT):
        delta = center_hz(i) - center_hz(i - 1)
        if delta != SX1276_FHSS_CHANNEL_SPACING_HZ:
            print(
                f"[FAIL] spacing at idx={i}: {delta} Hz "
                f"(expected {SX1276_FHSS_CHANNEL_SPACING_HZ})"
            )
            failures += 1
    for i in range(SX1276_FHSS_CHANNEL_COUNT):
        hz = center_hz(i)
        if not (SX1276_FHSS_BAND_LOWER_HZ + 750_000) <= hz <= (
            SX1276_FHSS_BAND_UPPER_HZ - 750_000
        ):
            print(f"[FAIL] edge-guard violated at idx={i}: {hz} Hz")
            failures += 1
    for bad in (50, 200, 255, -1):
        if center_hz(bad) != 0:
            print(f"[FAIL] OOR idx={bad} expected 0 got {center_hz(bad)}")
            failures += 1
    if failures == 0:
        print(
            f"[OK] fhss_chantab.py: all checks passed "
            f"(count={count()}, span={center_hz(0)}..{center_hz(49)} Hz)"
        )
        return 0
    print(f"[FAIL] fhss_chantab.py: {failures} check(s) failed")
    return 1


if __name__ == "__main__":
    raise SystemExit(_self_test())
