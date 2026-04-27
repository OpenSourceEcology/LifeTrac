"""Airtime ledger and image encode-mode ladder for the v25 base station.

The bridge and image pipeline can use this module to keep telemetry/image
traffic under the LoRa budget and to emit CMD_ENCODE_MODE only after sustained
pressure, avoiding one bad window flapping the encoder.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from lora_proto import (
    CMD_ENCODE_MODE,
    EncodeMode,
    PHY_BY_NAME,
    PHY_IMAGE,
    PHY_TELEMETRY,
    PhyProfile,
    encrypted_payload_len,
    lora_time_on_air_ms,
    pack_command,
)


@dataclass(frozen=True)
class AirtimeUtilization:
    image: float
    telemetry: float
    total: float


class RollingAirtimeLedger:
    """Track rolling airtime utilization by LoRa profile.

    Utilization is reported as a 0..1 fraction of wall time over ``window_ms``.
    Pass the cleartext frame length; AES-GCM nonce/tag overhead is added by
    default because that is what actually hits the radio.
    """

    def __init__(self, window_ms: int = 10_000) -> None:
        self.window_ms = window_ms
        self._events: deque[tuple[int, str, float]] = deque()

    def record(self, now_ms: int, profile: str | PhyProfile, cleartext_len: int,
               encrypted: bool = True) -> float:
        phy = self._resolve_profile(profile)
        payload_len = encrypted_payload_len(cleartext_len) if encrypted else cleartext_len
        airtime_ms = lora_time_on_air_ms(payload_len, phy)
        self._events.append((now_ms, phy.name, airtime_ms))
        self._trim(now_ms)
        return airtime_ms

    def utilization(self, now_ms: int) -> AirtimeUtilization:
        self._trim(now_ms)
        image_ms = 0.0
        telemetry_ms = 0.0
        total_ms = 0.0
        for _event_ms, profile, airtime_ms in self._events:
            total_ms += airtime_ms
            if profile == PHY_IMAGE.name:
                image_ms += airtime_ms
            elif profile == PHY_TELEMETRY.name:
                telemetry_ms += airtime_ms
        denom = float(self.window_ms)
        return AirtimeUtilization(
            image=image_ms / denom,
            telemetry=telemetry_ms / denom,
            total=total_ms / denom,
        )

    def _trim(self, now_ms: int) -> None:
        cutoff = now_ms - self.window_ms
        while self._events and self._events[0][0] < cutoff:
            self._events.popleft()

    @staticmethod
    def _resolve_profile(profile: str | PhyProfile) -> PhyProfile:
        if isinstance(profile, PhyProfile):
            return profile
        return PHY_BY_NAME[profile]


class EncodeModeController:
    """Three-window hysteresis for CMD_ENCODE_MODE decisions.

    Modes become more conservative as image utilization rises:
      full < 25 %, y_only < 50 %, motion_only < 80 %, wireframe otherwise.
    A candidate must be observed for ``required_windows`` consecutive windows
    before it becomes active.
    """

    def __init__(self, required_windows: int = 3) -> None:
        self.required_windows = required_windows
        self.mode = EncodeMode.FULL
        self._candidate = self.mode
        self._candidate_count = 0

    def observe(self, utilization: AirtimeUtilization) -> EncodeMode | None:
        target = self._target_for(utilization.image)
        if target == self.mode:
            self._candidate = target
            self._candidate_count = 0
            return None
        if target != self._candidate:
            self._candidate = target
            self._candidate_count = 1
            return None
        self._candidate_count += 1
        if self._candidate_count >= self.required_windows:
            self.mode = target
            self._candidate_count = 0
            return self.mode
        return None

    def command_frame(self, seq: int, mode: EncodeMode | None = None) -> bytes:
        selected = self.mode if mode is None else mode
        return pack_command(seq, CMD_ENCODE_MODE, bytes([int(selected)]))

    @staticmethod
    def _target_for(image_utilization: float) -> EncodeMode:
        if image_utilization >= 0.80:
            return EncodeMode.WIREFRAME
        if image_utilization >= 0.50:
            return EncodeMode.MOTION_ONLY
        if image_utilization >= 0.25:
            return EncodeMode.Y_ONLY
        return EncodeMode.FULL