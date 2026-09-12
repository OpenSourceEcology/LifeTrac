"""RS-12.11 (2026-09-12): when may the base transmit a command?

The base radio is half-duplex. A command transmission makes it deaf from
the RX disarm until the re-arm after TX_DONE, and a 100 ms image fragment
that overlaps that window in any way is not demodulated. The flash session
of 2026-09-12 (bench-evidence/RS_12_urc_counters_flash_session_2026-09-12)
put numbers on it: on the camera path 21 of 25 lost fragments sat 30–200 ms
after a base command TX (10.7x the received baseline), and the commands
responsible were the ones the idle-link drain fired on its 0.25 s poll
timeout — 250–300 ms after the last fragment, exactly where the next
2-fragment train's first fragment is due (train gaps 258–540 ms at 2 fps).

Two pure rules, kept out of the daemon so the bench can pin them:

* ``idle_drain_allowed`` — the idle drain may run only after the link has
  been quiet for ``quiet_s`` (no fragment at all). During a stream the only
  command path is the completion-aligned pump, which fires right after a
  train ends — the one instant a fragment is guaranteed not to be due.
  ``quiet_s <= 0`` restores the pre-RS-12.11 behaviour (bench A/B control).

* ``pump_window_open`` — a pump window opens when a frame completed OR when
  the train's last-index fragment arrived. Before RS-12.11 only a completed
  (published) frame opened the window, so a train that lost a fragment
  never opened one and its commands fell through to the idle drain.
"""
from __future__ import annotations


def idle_drain_allowed(now: float, last_frag_t: float, quiet_s: float) -> bool:
    """True when the idle-link drain may transmit queued commands."""
    if quiet_s <= 0.0:
        return True
    return (now - last_frag_t) >= quiet_s


def pump_window_open(frame_done: bool, train_end: bool) -> bool:
    """True when the completion-aligned pump may send one command copy."""
    return bool(frame_done or train_end)
