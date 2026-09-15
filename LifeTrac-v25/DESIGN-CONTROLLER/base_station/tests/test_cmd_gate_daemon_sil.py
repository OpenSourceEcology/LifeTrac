"""RS-12.14 — daemon-level SIL for the shared send gate's two counters.

PR #124 review (round 5): the pure `cmd_timing` cases pin the retry-time
comparison, but the metric that the flight legs report is produced by the
DAEMON — `ImageRxDaemon._ctrl_due()` (the mutation-free look-ahead) and
`_cmd_gate_open(now, wants_send)` (which scores `cmd_gate_held` on every
closed-gate check and `cmd_gate_deferred` only when a send was really due).
Those two were coverable only through a live leg, so a regression could
restore the false-positive/undercount behaviour this instrumentation exists
to fix and still pass the suite. These cases drive them directly.

Contract pinned here:
  * `_ctrl_due` is TRUE only for work that `_next_ctrl_body` would actually
    dispatch now: a pending entry inside its attempt budget, inside its
    deadline, and past its retry gap — or a legacy entry queued.
  * `_ctrl_due` mutates nothing (a look-ahead, not a pop).
  * a CLOSED gate always books `cmd_gate_held`; it books `cmd_gate_deferred`
    only when the caller passed `wants_send`.
  * an OPEN gate books neither.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

OPCODE = 0x60          # REQ_KEYFRAME, arbitrary — the map is keyed by opcode
NOW = 10_000.0


def _daemon():
    """A constructed-but-never-connected daemon (same pattern as
    test_image_tx_rx_optimization): `loop://` opens no hardware and the
    MQTT client is not started, so only the pure decision paths run."""
    from image_rx_daemon import ImageRxDaemon
    return ImageRxDaemon(uart="loop://", baud="0", mqtt_host="127.0.0.1",
                         mqtt_port=1883, reassembler_timeout_ms=1500)


def _pending(d, *, attempts=0, last_send=0.0, deadline_in=30.0,
             max_attempts=20, now=NOW):
    d._pending_cmds[OPCODE] = {
        "body": b"\x01", "attempts": attempts, "t0": now,
        "last_send": last_send, "deadline": now + deadline_in,
        "max_attempts": max_attempts,
    }


class CtrlDueLookAhead(unittest.TestCase):
    def setUp(self) -> None:
        self.d = _daemon()

    def test_idle_daemon_has_nothing_due(self) -> None:
        self.assertFalse(self.d._ctrl_due(NOW))

    def test_fresh_pending_is_due(self) -> None:
        _pending(self.d)                      # attempts 0 => retry gap passed
        self.assertTrue(self.d._ctrl_due(NOW))

    def test_pending_inside_backoff_is_not_due(self) -> None:
        _pending(self.d, attempts=1, last_send=NOW)
        self.assertFalse(self.d._ctrl_due(NOW))

    def test_pending_past_its_backoff_is_due_again(self) -> None:
        _pending(self.d, attempts=1, last_send=NOW - 60.0)
        self.assertTrue(self.d._ctrl_due(NOW))

    def test_exhausted_attempts_not_due(self) -> None:
        _pending(self.d, attempts=20, max_attempts=20)
        self.assertFalse(self.d._ctrl_due(NOW))

    def test_expired_deadline_not_due(self) -> None:
        _pending(self.d, deadline_in=-1.0)
        self.assertFalse(self.d._ctrl_due(NOW))

    def test_queued_legacy_entry_is_due(self) -> None:
        self.d._ctrl_out.put_nowait(b"\x02legacy")
        self.assertTrue(self.d._ctrl_due(NOW))

    def test_look_ahead_does_not_consume(self) -> None:
        """The leg-N miscount came from a look-ahead that mutated state."""
        _pending(self.d)
        self.d._ctrl_out.put_nowait(b"\x02legacy")
        for _ in range(5):
            self.assertTrue(self.d._ctrl_due(NOW))
        self.assertEqual(self.d._pending_cmds[OPCODE]["attempts"], 0)
        self.assertEqual(self.d._pending_cmds[OPCODE]["last_send"], 0.0)
        self.assertEqual(self.d._ctrl_out.qsize(), 1)


class GateCounters(unittest.TestCase):
    def setUp(self) -> None:
        self.d = _daemon()

    def _held(self):
        return getattr(self.d, "_cmd_gate_held", 0)

    def _deferred(self):
        return getattr(self.d, "_cmd_gate_deferred", 0)

    def test_open_gate_books_neither(self) -> None:
        self.d._last_cmd_send_t = NOW - 100.0          # long past any gap
        self.assertTrue(self.d._cmd_gate_open(NOW, True))
        self.assertEqual((self._held(), self._deferred()), (0, 0))

    def test_closed_gate_with_send_due_books_both(self) -> None:
        self.d._last_cmd_send_t = NOW                  # just sent => closed
        self.assertFalse(self.d._cmd_gate_open(NOW, True))
        self.assertEqual((self._held(), self._deferred()), (1, 1))

    def test_closed_gate_without_send_due_books_held_only(self) -> None:
        """The leg-N caveat: 524 closed-gate checks were NOT 524 deferrals."""
        self.d._last_cmd_send_t = NOW
        for _ in range(7):
            self.assertFalse(self.d._cmd_gate_open(NOW, False))
        self.assertEqual((self._held(), self._deferred()), (7, 0))

    def test_default_wants_send_is_false(self) -> None:
        self.d._last_cmd_send_t = NOW
        self.assertFalse(self.d._cmd_gate_open(NOW))
        self.assertEqual((self._held(), self._deferred()), (1, 0))

    def test_pump_shaped_sequence(self) -> None:
        """Pump cadence faster than the gate, with a command due only part
        of the time: held counts every closed check, deferred only the due
        ones — the distinction the rerun is meant to measure."""
        self.d._last_cmd_send_t = NOW
        for i in range(10):
            due = (i % 2 == 0)
            if due:
                _pending(self.d)
            else:
                self.d._pending_cmds.clear()
            self.d._cmd_gate_open(NOW, self.d._ctrl_due(NOW))
        self.assertEqual(self._held(), 10)
        self.assertEqual(self._deferred(), 5)


if __name__ == "__main__":
    unittest.main()
