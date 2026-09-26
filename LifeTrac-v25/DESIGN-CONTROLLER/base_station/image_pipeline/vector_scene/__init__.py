"""VS1 vector scene: the shared codec for ``TileDeltaFrame`` codec 6.

See ``DESIGN-CONTROLLER/VECTOR_SCENE.md`` §3 for the wire format this package
implements. Everything here is pure stdlib so the tractor encoder, the base
store, ``web_ui`` and the unit tests can import it without numpy, PIL or MQTT.
"""
from .codec import (  # noqa: F401
    CODEC_VECTOR, HEADER_BITS, STATIC_PALETTE, VS_VERSION,
    Anom, Blob, CalRev, Confirm, Del, Digest, Edge, Fill, Frame, FrameTooLarge,
    Gain, Gshift, Gzoom, Header, Hole, HznAbs, HznColours, HznNoHorizon,
    HznResid, Insert, LayerClear, Pal, Poly, Skyline, Status, Tree, Ucol, Upd,
    VFill, VsDecodeError, crc8, crc16_ccitt_false, decode_frame, define_hash,
    digest_crc, encode_frame, kraft_sum, pack_record, record_bits, state_hash,
)
