"""Operator-facing feedback contract for build-config installs (Round 29b-beta / BC-10).

The X8-side installer (:mod:`installer_daemon`) and the watcher loop
(:mod:`config_watcher`) both produce :class:`installer_daemon.InstallResult`
objects (or watcher events). This module is the *single source of truth*
for how those outcomes are surfaced to the human standing next to the
tractor with a USB stick:

* a one-line OLED status string (max :data:`OLED_LINE_MAX` chars), and
* a blink pattern for the status LED.

Pinning the table here keeps the OLED text stable across firmware
revisions (so an operator who's learned "3 long blinks = applied" can
keep that mental model) and lets the SIL gate assert against a single
canonical mapping rather than two scattered string literals.

LED conventions
---------------

The status LED is one bicolour LED on the X8 side panel. Patterns are
described as a sequence of (state, milliseconds) tuples. ``on`` /
``off`` / ``red`` / ``green`` / ``amber`` are the abstract states the
hardware driver translates to GPIO writes. Patterns repeat until the
next status transition.

* ``applied``   : 3 long green flashes (700 ms on, 300 ms off) then
  steady green for 5 s.
* ``rejected``  : 3 short red flashes (150 ms on, 150 ms off) then
  steady red for 5 s.
* ``deferred``  : amber slow blink (500/500) until quiescence clears.
* ``noop``      : single short green pulse (200 ms).
* ``idle``      : LED off (no recent install).

OLED text discipline
--------------------

The OLED line is a single 21-character row (the same row used for the
LoRa link banner; see ``LORA_IMPLEMENTATION.md`` for the budget). We
abbreviate aggressively -- ``cfg ok`` is more useful at a glance than
``Build configuration applied successfully``. The reason string from
the install result is appended only when there's room.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import installer_daemon

OLED_LINE_MAX = 21
"""Hard width budget for one OLED status row (matches LoRa banner row)."""


@dataclass(frozen=True)
class LedPattern:
    """Abstract LED blink pattern. Hardware-agnostic on purpose.

    ``steps`` is a sequence of ``(state, milliseconds)`` pairs. The
    driver replays the sequence; if ``repeat`` is True it loops until
    the next transition, otherwise it runs once and then idles.
    """

    name: str
    steps: tuple[tuple[str, int], ...]
    repeat: bool = False


# Pinned LED patterns. Tests assert the steps tuple verbatim, so any
# change here is loud (intentional contract change, not a tweak).
LED_APPLIED = LedPattern(
    name="applied",
    steps=(
        ("green", 700), ("off", 300),
        ("green", 700), ("off", 300),
        ("green", 700), ("off", 300),
        ("green", 5000),
    ),
    repeat=False,
)
LED_REJECTED = LedPattern(
    name="rejected",
    steps=(
        ("red", 150), ("off", 150),
        ("red", 150), ("off", 150),
        ("red", 150), ("off", 150),
        ("red", 5000),
    ),
    repeat=False,
)
LED_DEFERRED = LedPattern(
    name="deferred",
    steps=(("amber", 500), ("off", 500)),
    repeat=True,
)
LED_NOOP = LedPattern(
    name="noop",
    steps=(("green", 200),),
    repeat=False,
)
LED_IDLE = LedPattern(
    name="idle",
    steps=(("off", 1000),),
    repeat=True,
)

# Status -> (LED pattern, OLED prefix). Prefix is what the operator
# reads first; the trailing reason is appended by feedback_for() when
# it fits.
STATUS_TABLE: dict[str, tuple[LedPattern, str]] = {
    installer_daemon.STATUS_APPLIED:  (LED_APPLIED,  "cfg ok"),
    installer_daemon.STATUS_REJECTED: (LED_REJECTED, "cfg REJECTED"),
    installer_daemon.STATUS_NOOP:     (LED_NOOP,     "cfg same"),
    installer_daemon.STATUS_DEFERRED: (LED_DEFERRED, "cfg deferred"),
}


@dataclass(frozen=True)
class Feedback:
    """Bundle of outputs the X8 surfaces for one install outcome."""

    status: str
    led: LedPattern
    oled_line: str

    def __post_init__(self) -> None:
        if len(self.oled_line) > OLED_LINE_MAX:
            raise ValueError(
                f"oled_line {self.oled_line!r} exceeds {OLED_LINE_MAX}-char budget"
            )


def _truncate(text: str, budget: int) -> str:
    if len(text) <= budget:
        return text
    if budget <= 1:
        return text[:budget]
    return text[: budget - 1] + "\u2026"  # ellipsis


def feedback_for(result: installer_daemon.InstallResult) -> Feedback:
    """Map an install result to the OLED line + LED pattern to drive.

    Stable: the SIL gate locks the (status -> LED, OLED-prefix) table
    so changing it is a deliberate contract bump.
    """
    if result.status not in STATUS_TABLE:
        raise ValueError(
            f"feedback_for: unknown status {result.status!r}; "
            f"expected one of {tuple(STATUS_TABLE)!r}"
        )
    led, prefix = STATUS_TABLE[result.status]
    # Always include sha8 when we have one -- operator can correlate
    # the LED with the result.json on the stick at a glance.
    sha8 = (result.applied_sha or "")[:8]
    if sha8:
        head = f"{prefix} {sha8}"
    else:
        head = prefix
    head = _truncate(head, OLED_LINE_MAX)
    return Feedback(status=result.status, led=led, oled_line=head)


def feedback_for_status(status: str, *, sha8: str | None = None) -> Feedback:
    """Direct accessor used by callers that don't have an InstallResult.

    The watcher loop (:mod:`config_watcher`) uses this to surface a
    deferred-pending-quiescence banner before any install attempt.
    """
    if status not in STATUS_TABLE:
        raise ValueError(
            f"feedback_for_status: unknown status {status!r}; "
            f"expected one of {tuple(STATUS_TABLE)!r}"
        )
    led, prefix = STATUS_TABLE[status]
    head = f"{prefix} {sha8}" if sha8 else prefix
    return Feedback(status=status, led=led, oled_line=_truncate(head, OLED_LINE_MAX))


__all__ = [
    "Feedback",
    "LED_APPLIED",
    "LED_DEFERRED",
    "LED_IDLE",
    "LED_NOOP",
    "LED_REJECTED",
    "LedPattern",
    "OLED_LINE_MAX",
    "STATUS_TABLE",
    "feedback_for",
    "feedback_for_status",
]
