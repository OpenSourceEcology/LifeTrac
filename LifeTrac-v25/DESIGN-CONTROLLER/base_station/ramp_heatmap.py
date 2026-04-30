"""Round 53 / K-E2: web UI ramp-state heatmap data model.

Operator-visible "why did my stop feel mushy" \u2014 a per-axis,
per-tick classification of the relationship between the operator's
stick intent (``raw``) and the post-ramp value the firmware actually
asserts on the coils (``effective``). Roadmap entry K-E2.

This module is **pure data transformation**. It does not own the
firmware ramp math (see
[tests/test_axis_ramp_sil.py](tests/test_axis_ramp_sil.py) for the
canonical mirror) and it does not render anything on its own \u2014
the web UI consumes the JSON-serializable rows produced by
:func:`build_heatmap` and paints them as a side-by-side raw / effective
bar with a per-tick state-coloured underlay.

Five classification states, in precedence order:

* ``"idle"`` \u2014 both ``raw`` and ``effective`` are within the
  deadband. The axis is at rest; nothing to display.
* ``"matched"`` \u2014 ``|raw - effective| <= deadband``. The
  effective bar tracks the operator one-for-one (steady-state
  pass-through, BC-21 mix-then-ramp at the asymptote).
* ``"reversal"`` \u2014 ``raw`` and ``effective`` are both outside
  the deadband but have opposite signs. The BC-22 reversal-brake
  decay is in flight; the lag is *expected* and the operator
  should see a clear "stop first" indication, not a mushiness flag.
* ``"decay"`` \u2014 ``raw`` is within the deadband but ``|effective|``
  is outside it. A release ramp is in flight (BC-21 / K-A4).
  Same-sign lag with the operator commanding stop.
* ``"mushy"`` \u2014 same-sign lag where ``|raw - effective| >
  mushy_threshold``. The effective value is trailing the raw command
  by more than the operator should feel comfortable with. This is
  the cell the operator is meant to *see* in the heatmap.

The default ``deadband`` mirrors the firmware ``AXIS_DEADBAND`` (BC-29
``ui.axis_deadband`` leaf, default 13). The default
``mushy_threshold`` of 40 (\u2248 31% of int8 full scale) is a
conservative starting point intended to catch obvious lag without
flagging every BC-26 S-curve release.

Pure stdlib; safe to import from web_ui.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

# Identity defaults: these mirror firmware constants. Both are
# overridable per-call so a build that widens BC-29 ui.axis_deadband
# can pass the wider value through without recompiling this module.
DEFAULT_DEADBAND = 13
DEFAULT_MUSHY_THRESHOLD = 40

# Public state vocabulary. Pinned by the SIL so a regression that
# silently renames a state trips here.
STATES = ("idle", "matched", "reversal", "decay", "mushy")


@dataclass(frozen=True)
class HeatmapSample:
    """One (raw, effective) pair at a wall-clock instant.

    ``raw`` is the post-mix / post-stick-curve / pre-ramp axis intent
    (int8 range, ``-127..127``). ``effective`` is the post-ramp value
    asserted on the coils (same range). ``now_ms`` is the SIL stand-in
    for ``millis()`` and is preserved verbatim into the output rows so
    the web UI can align cells against the time axis.
    """
    now_ms: int
    raw: int
    effective: int


def lag(raw: int, effective: int) -> int:
    """Signed lag = ``raw - effective``.

    Positive when the operator is asking for more than the firmware is
    asserting (typical pursuit ramp). Negative when the firmware is
    asserting more than the operator is currently asking for (typical
    release / decay ramp). Used by both :func:`classify_state` and the
    heatmap row to give the web UI an unambiguous numeric to colour.
    """
    return int(raw) - int(effective)


def classify_state(raw: int, effective: int, *,
                   deadband: int = DEFAULT_DEADBAND,
                   mushy_threshold: int = DEFAULT_MUSHY_THRESHOLD) -> str:
    """Return one of :data:`STATES` for the given (raw, effective) pair.

    Precedence (top wins):

    1. ``idle`` if both inputs are within ``deadband``.
    2. ``reversal`` if both are outside ``deadband`` and have opposite
       signs (BC-22 territory \u2014 lag is expected and not mushy).
    3. ``decay`` if ``raw`` is inside the deadband but ``|effective|``
       is outside (release ramp in flight).
    4. ``mushy`` if ``|raw - effective| > mushy_threshold``
       (same-sign pursuit lag exceeding the comfort threshold).
    5. ``matched`` otherwise (steady-state or small in-flight lag).

    ``deadband`` and ``mushy_threshold`` are validated as non-negative
    ints; values outside that range raise :class:`ValueError`.
    """
    if deadband < 0:
        raise ValueError(f"deadband must be >= 0, got {deadband!r}")
    if mushy_threshold < 0:
        raise ValueError(
            f"mushy_threshold must be >= 0, got {mushy_threshold!r}")
    raw_active = (raw > deadband) or (raw < -deadband)
    eff_active = (effective > deadband) or (effective < -deadband)
    if not raw_active and not eff_active:
        return "idle"
    if raw_active and eff_active and ((raw > 0) != (effective > 0)):
        return "reversal"
    if not raw_active and eff_active:
        return "decay"
    delta = lag(raw, effective)
    if delta > mushy_threshold or delta < -mushy_threshold:
        return "mushy"
    return "matched"


def build_heatmap_row(sample: HeatmapSample, *,
                      deadband: int = DEFAULT_DEADBAND,
                      mushy_threshold: int = DEFAULT_MUSHY_THRESHOLD) -> dict:
    """Render one :class:`HeatmapSample` as a JSON-serializable dict.

    The web UI reads four keys per row: ``now_ms``, ``raw``,
    ``effective``, ``lag``, and ``state``. ``lag`` is the signed
    :func:`lag` value (so the painter can offset the effective bar
    against the raw bar without recomputing). ``state`` is the
    :func:`classify_state` result so the painter can colour the cell.
    """
    return {
        "now_ms": int(sample.now_ms),
        "raw": int(sample.raw),
        "effective": int(sample.effective),
        "lag": lag(sample.raw, sample.effective),
        "state": classify_state(sample.raw, sample.effective,
                                deadband=deadband,
                                mushy_threshold=mushy_threshold),
    }


def build_heatmap(streams: dict[str, Iterable[HeatmapSample]], *,
                  deadband: int = DEFAULT_DEADBAND,
                  mushy_threshold: int = DEFAULT_MUSHY_THRESHOLD) -> dict:
    """Render per-axis sample streams as a JSON-serializable heatmap.

    ``streams`` is a mapping from axis name (e.g. ``"left_track"``,
    ``"right_track"``, ``"arms"``, ``"bucket"``) to an iterable of
    :class:`HeatmapSample` records, ordered by ``now_ms``. Returns a
    dict with two top-level keys:

    * ``"deadband"`` and ``"mushy_threshold"`` \u2014 the thresholds
      used so the painter can render legend tick marks consistent with
      the actual classification.
    * ``"axes"`` \u2014 a dict mapping axis name to the per-row list of
      :func:`build_heatmap_row` outputs.

    Stream order inside each axis is preserved verbatim; the function
    does not sort or deduplicate.
    """
    return {
        "deadband": int(deadband),
        "mushy_threshold": int(mushy_threshold),
        "axes": {
            str(name): [build_heatmap_row(s, deadband=deadband,
                                          mushy_threshold=mushy_threshold)
                        for s in samples]
            for name, samples in streams.items()
        },
    }
