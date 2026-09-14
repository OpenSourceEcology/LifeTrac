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


# RS-12.14 (2026-09-12): the keyframe self-heal storm on FHSS. A pending
# (ack-driven) command retried every 0.4 s on every pump window until acked,
# and every re-trigger extended its deadline; on profile 1 the acks mostly
# do not come back (reverse-path delivery was 1 of 17 even on a healthy
# leg), so one perpetually refreshed REQ_KEYFRAME fired 222 times in 300 s,
# each a base TX that skips the FHSS follower under tx-busy, and the leg
# collapsed from 2.1 % to 62.8 % loss (RS_12_12 legs H vs I). Three pure
# rules bound that:
#   pending_retry_gap  - exponential backoff between retries of one command
#   giveup_cooldown_active - after a command gives up, the same opcode is
#                        refused for a cool-down instead of restarting
#   pump_min_gap       - while a stream is active, ANY two base commands
#                        are at least CMD_STREAM_MIN_GAP_S apart

def pending_retry_gap(attempts: int, base_gap_s: float, factor: float,
                      cap_s: float) -> float:
    """Gap to wait before retry number `attempts + 1` (attempts already made)."""
    if attempts <= 0:
        return 0.0
    gap = base_gap_s * (max(factor, 1.0) ** (attempts - 1))
    return min(gap, cap_s)


def giveup_cooldown_active(now: float, giveup_at: float, cooldown_s: float) -> bool:
    """True while a given-up opcode must not be re-registered."""
    return cooldown_s > 0.0 and giveup_at > 0.0 and (now - giveup_at) < cooldown_s


def pump_min_gap(stream_active: bool, stream_gap_s: float,
                 idle_gap_s: float = 0.12) -> float:
    """Minimum spacing between two pump sends: the stream gap while fragments
    are flowing (each base TX costs the FHSS follower), the old 120 ms
    copy-spacing otherwise."""
    return max(stream_gap_s, idle_gap_s) if stream_active else idle_gap_s


# PR #121 review (2026-09-14): the stream gap guarded only the aligned pump.
# Profile switches (two immediate copies), the CONF, reactive probes and the
# idle drain each sent on their own clock - leg J recorded two idle-drain
# sends 60 ms apart during a lock loss. One shared gate now decides every
# dispatch on every path, measured from the previous dispatch's last on-air
# copy. A closed gate is counted and re-asked on the next pass, never waited
# on: the RX loop must keep servicing the modem FIFO.

def send_gate_open(now: float, last_send_t: float, stream_active: bool,
                   stream_gap_s: float, idle_gap_s: float = 0.12) -> bool:
    """True when ANY command path may dispatch one command copy."""
    return (now - last_send_t) >= pump_min_gap(stream_active, stream_gap_s,
                                               idle_gap_s)


# PR #124 review (2026-09-14): `cmd_gate_held` counts every closed-gate
# CHECK, before the daemon knows whether any command is actually due, so
# it cannot by itself show the gate deferring a send. This look-ahead is
# the mutation-free half of the pending retry rule: is retry number
# `attempts + 1` due at `now`? The daemon uses it to score a closed gate
# as a real deferral (`cmd_gate_deferred`) only when a command was due.

def pending_retry_due(now: float, last_send_t: float, attempts: int,
                      base_gap_s: float, factor: float, cap_s: float) -> bool:
    """True when a pending command's next retry is due at `now`."""
    return (now - last_send_t) >= pending_retry_gap(attempts, base_gap_s,
                                                    factor, cap_s)
