"""Pre-fragmenter for encoded TileDeltaFrame payloads.

The H747 M7 normally chops the X8's encoded payload into ≤25 ms-airtime
LoRa fragments, but two paths need the chunker on the X8 side:

1. **Self-test / replay mode** — running ``camera_service.py`` against a
   captured image directory, where there is no M7 to do fragmentation.
2. **High-FPS Q/P degraded modes** — the X8 already knows the upcoming
   PHY profile (image vs. retuned-telemetry) and can pre-budget how many
   tiles to encode this frame so the M7 isn't blocked sizing fragments.

Wire format reuses the project-wide ``TELEMETRY_FRAGMENT_MAGIC`` (0xFE)
header that ``base_station.image_pipeline.reassemble`` already understands
— image fragments and oversized-telemetry fragments share one reassembly
path on the base station.

Public API
----------

- ``pack_image_fragments(payload, frag_seq, *, profile=PHY_IMAGE,
  max_air_ms=25.0)`` — list of fragment bodies (caller wraps each in a
  TelemetryFrame on topic ``0x25``).
- ``estimate_fragment_count(payload_len, *, profile=PHY_IMAGE,
  max_air_ms=25.0)`` — used by the encoder's quality controller so it
  can drop tiles before WebP runs.

Both helpers are thin wrappers around ``lora_proto.pack_telemetry_fragments``
to guarantee the X8 and the base station stay byte-identical.
"""
from __future__ import annotations

from lora_proto import (   # noqa: E402  (sys.path is configured by the runner)
    PHY_IMAGE,
    PhyProfile,
    TELEMETRY_FRAGMENT_HEADER_LEN,
    TELEMETRY_FRAGMENT_MAGIC,
    TELEMETRY_FRAGMENT_MAX_AIRTIME_MS,
    max_telemetry_fragment_payload,
    pack_telemetry_fragments,
)

# Re-export so callers don't have to reach into base_station from this side.
IMAGE_FRAGMENT_MAGIC = TELEMETRY_FRAGMENT_MAGIC
IMAGE_FRAGMENT_HEADER_LEN = TELEMETRY_FRAGMENT_HEADER_LEN
IMAGE_FRAGMENT_MAX_AIRTIME_MS = TELEMETRY_FRAGMENT_MAX_AIRTIME_MS


def pack_image_fragments(payload: bytes,
                         frag_seq: int,
                         *,
                         profile: PhyProfile = PHY_IMAGE,
                         max_air_ms: float = TELEMETRY_FRAGMENT_MAX_AIRTIME_MS) -> list[bytes]:
    """Split an encoded TileDeltaFrame into ≤``max_air_ms`` fragment bodies.

    Returns the list of fragment bodies (header + data). Each body is
    wrapped by the caller into a TelemetryFrame on topic ``0x25`` before
    going on the air. ``frag_seq`` rolls 0..255 per logical image frame
    and lets the reassembler distinguish back-to-back frames.
    """
    return pack_telemetry_fragments(payload, frag_seq, profile, max_air_ms=max_air_ms)


def estimate_fragment_count(payload_len: int,
                            *,
                            profile: PhyProfile = PHY_IMAGE,
                            max_air_ms: float = TELEMETRY_FRAGMENT_MAX_AIRTIME_MS) -> int:
    """How many fragments a payload of ``payload_len`` bytes would produce.

    Used by the X8's quality controller to back off WebP quality / tile
    count *before* paying the encode cost when it can already see the
    payload won't fit the budget.
    """
    if payload_len <= 0:
        return 0
    chunk = max_telemetry_fragment_payload(profile, max_air_ms)
    if chunk <= 0:
        raise ValueError(
            f"profile {profile.name} cannot fit any fragment in {max_air_ms} ms")
    return max(1, (payload_len + chunk - 1) // chunk)


def max_payload_for_n_fragments(n_fragments: int,
                                *,
                                profile: PhyProfile = PHY_IMAGE,
                                max_air_ms: float = TELEMETRY_FRAGMENT_MAX_AIRTIME_MS) -> int:
    """Inverse of :func:`estimate_fragment_count`.

    Returns the maximum logical payload size (in bytes) that fits inside
    ``n_fragments`` fragments at the given PHY profile and per-fragment
    airtime cap. The encoder uses this as its byte budget.
    """
    if n_fragments <= 0:
        return 0
    chunk = max_telemetry_fragment_payload(profile, max_air_ms)
    return chunk * n_fragments
