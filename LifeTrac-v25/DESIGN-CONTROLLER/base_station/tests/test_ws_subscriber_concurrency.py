"""Round 12 — WS subscriber concurrency stress test.

Companion to the IP-202 / IP-208 admission-cap unit tests in
`test_web_ui_validation.py` and a follow-up to Round 9 §B (control-cap)
and §C (`_subscribers_lock` + `_snapshot_subscribers`).

The single-threaded happy-path tests already prove: cap rejects with
4429, control-WS demands a session, `_subscribers_lock` exists. What
they don't prove is the property §C was actually introduced for —
that the MQTT background thread can iterate the WS pools while the
FastAPI event loop is mutating them, *without* `RuntimeError: Set
changed size during iteration` ever firing, and *without* any pool
exceeding its cap under concurrent admit attempts.

This suite drives `_admit_ws`, `_discard_subscriber`,
`_snapshot_subscribers`, and the MQTT-thread fan-out (`_on_mqtt_message`)
together at high churn using a real asyncio loop on a background
thread — exactly the topology the production gateway runs.

Properties exercised (WC-A … WC-G):

- WC-A  Cap is honoured under N concurrent admits (no over-admit race).
- WC-B  Repeated admit/discard cycles never raise and never leak slots.
- WC-C  `_snapshot_subscribers()` callers can iterate the snapshot
        list-free of "Set changed size during iteration" while another
        thread mutates the pool at 1 kHz.
- WC-D  `_on_mqtt_message()` survives the WS pool being mutated under
        its feet — telemetry messages are delivered to whichever
        subscribers are present at snapshot time, never raise.
- WC-E  Each pool's cap is independent — saturating telemetry must not
        block image/state/control admissions.
- WC-F  After a churn cycle, all pools fully drain (no leaked WSes) and
        the cap counter is correct on the next fill.
- WC-G  Over-cap rejections invoke `WebSocket.close(code=4429)`.
"""

from __future__ import annotations

import asyncio
import os
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

try:
    import paho.mqtt.client  # noqa: F401
    import fastapi           # noqa: F401
except ImportError:                                        # pragma: no cover
    raise unittest.SkipTest("paho-mqtt + fastapi required for web_ui tests")


# --------------------------------------------------------------------------
# Fake WebSocket — mimics the starlette.websockets.WebSocket surface that
# `_admit_ws`, `_discard_subscriber`, and `_on_mqtt_message` actually
# touch. We don't need a real socket; we need an awaitable that records
# accept/close/send_text/send_bytes calls.
# --------------------------------------------------------------------------
class FakeWS:
    __slots__ = ("accepted", "closed", "close_code", "sent_text",
                 "sent_bytes", "_lock")

    def __init__(self) -> None:
        self.accepted = False
        self.closed = False
        self.close_code: int | None = None
        self.sent_text = 0
        self.sent_bytes = 0
        # Guards counters because send_* may be called from many tasks.
        self._lock = threading.Lock()

    async def accept(self) -> None:
        self.accepted = True

    async def close(self, code: int = 1000) -> None:
        self.closed = True
        self.close_code = code

    async def send_text(self, text: str) -> None:
        if self.closed:
            raise RuntimeError("send on closed ws")
        with self._lock:
            self.sent_text += 1

    async def send_bytes(self, blob: bytes) -> None:
        if self.closed:
            raise RuntimeError("send on closed ws")
        with self._lock:
            self.sent_bytes += 1


def _bootstrap_web_ui():
    os.environ["LIFETRAC_PIN"] = "424242"
    os.environ["LIFETRAC_ALLOW_UNCONFIGURED_KEY"] = "1"
    with mock.patch("paho.mqtt.client.Client") as mqtt_class:
        instance = mqtt_class.return_value
        instance.connect = mock.MagicMock()
        instance.loop_start = mock.MagicMock()
        instance.subscribe = mock.MagicMock()
        instance.publish = mock.MagicMock()
        import importlib
        import web_ui
        importlib.reload(web_ui)
        return web_ui, instance


class _LoopFixture:
    """Run an asyncio loop on a background thread so we can submit
    coroutines via `run_coroutine_threadsafe`, the same pattern
    `_on_mqtt_message` uses against `app.state.loop`."""

    def __init__(self) -> None:
        self.loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="ws-stress-loop")
        self._thread.start()
        # Wait for loop to actually be running before tests submit work.
        ready = threading.Event()
        self.loop.call_soon_threadsafe(ready.set)
        ready.wait(timeout=2.0)

    def _run(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def stop(self) -> None:
        self.loop.call_soon_threadsafe(self.loop.stop)
        self._thread.join(timeout=2.0)
        self.loop.close()


class WSSubscriberConcurrencyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.web_ui, cls.mqtt = _bootstrap_web_ui()
        cls.lf = _LoopFixture()
        cls.web_ui.app.state.loop = cls.lf.loop

    @classmethod
    def tearDownClass(cls) -> None:
        cls.lf.stop()

    def setUp(self) -> None:
        # Each test starts from clean pools.
        with self.web_ui._subscribers_lock:
            self.web_ui.telemetry_subscribers.clear()
            self.web_ui.image_subscribers.clear()
            self.web_ui.state_subscribers.clear()
            self.web_ui.control_subscribers.clear()

    # ---- helpers --------------------------------------------------------
    def _admit(self, ws: FakeWS, pool, cap: int, label: str) -> bool:
        return self.lf.submit(
            self.web_ui._admit_ws(ws, pool, cap, label)
        ).result(timeout=5.0)

    def _discard(self, ws: FakeWS, pool) -> None:
        # Synchronous helper — no need for the loop.
        self.web_ui._discard_subscriber(pool, ws)

    # ---- WC-A -----------------------------------------------------------
    def test_wc_a_cap_honoured_under_concurrent_admits(self) -> None:
        cap = self.web_ui.MAX_TELEMETRY_SUBSCRIBERS
        N = cap * 4  # heavy oversubscribe
        wses = [FakeWS() for _ in range(N)]
        pool = self.web_ui.telemetry_subscribers

        # Fan out admit calls onto the loop in parallel — they all share
        # the same `_subscribers_lock`, so we're stress-testing that the
        # check-then-add window can't admit past the cap.
        futs = [
            self.lf.submit(self.web_ui._admit_ws(w, pool, cap, "telemetry"))
            for w in wses
        ]
        results = [f.result(timeout=5.0) for f in futs]

        admitted = sum(1 for r in results if r)
        rejected = sum(1 for r in results if not r)
        self.assertEqual(admitted, cap,
                         f"admitted {admitted}, expected exactly cap={cap}")
        self.assertEqual(rejected, N - cap)
        with self.web_ui._subscribers_lock:
            self.assertEqual(len(pool), cap)

        # WC-G: every rejection must close with the standard 4429 code.
        rejected_wses = [w for w, r in zip(wses, results) if not r]
        for w in rejected_wses:
            self.assertTrue(w.closed, "rejected ws must be closed")
            self.assertEqual(w.close_code,
                             self.web_ui.WS_OVER_CAPACITY_CODE)

        # And every admitted ws must be `accept()`-ed but not closed.
        admitted_wses = [w for w, r in zip(wses, results) if r]
        for w in admitted_wses:
            self.assertTrue(w.accepted)
            self.assertFalse(w.closed)

    # ---- WC-B -----------------------------------------------------------
    def test_wc_b_admit_discard_cycles_no_leak(self) -> None:
        cap = self.web_ui.MAX_CONTROL_SUBSCRIBERS
        pool = self.web_ui.control_subscribers
        for _ in range(200):
            ws = FakeWS()
            ok = self._admit(ws, pool, cap, "control")
            self.assertTrue(ok)
            with self.web_ui._subscribers_lock:
                self.assertLessEqual(len(pool), cap)
            self._discard(ws, pool)
            with self.web_ui._subscribers_lock:
                self.assertNotIn(ws, pool)
        with self.web_ui._subscribers_lock:
            self.assertEqual(len(pool), 0)

    # ---- WC-C -----------------------------------------------------------
    def test_wc_c_snapshot_iteration_safe_under_churn(self) -> None:
        """The whole point of `_snapshot_subscribers` + `_subscribers_lock`:
        iterating the *snapshot* must never see a mid-mutation set."""
        cap = self.web_ui.MAX_TELEMETRY_SUBSCRIBERS
        pool = self.web_ui.telemetry_subscribers

        stop = threading.Event()
        errors: list[BaseException] = []

        def churn() -> None:
            try:
                while not stop.is_set():
                    ws = FakeWS()
                    if self._admit(ws, pool, cap, "telemetry"):
                        # Brief residency, then leave.
                        self._discard(ws, pool)
            except BaseException as exc:                  # pragma: no cover
                errors.append(exc)

        def reader() -> None:
            try:
                # Loop hard for a fixed iteration count rather than
                # wall-clock — keeps the suite fast and deterministic.
                for _ in range(20_000):
                    snap = self.web_ui._snapshot_subscribers(pool)
                    # Iterate to provoke "Set changed size" if the snapshot
                    # were a live view rather than a list copy.
                    for _ws in snap:
                        pass
            except BaseException as exc:
                errors.append(exc)

        churners = [threading.Thread(target=churn, daemon=True)
                    for _ in range(4)]
        readers = [threading.Thread(target=reader, daemon=True)
                   for _ in range(2)]
        for t in churners + readers:
            t.start()
        # Readers terminate on their own iteration count; then signal
        # churners to stop.
        for t in readers:
            t.join(timeout=15.0)
            self.assertFalse(t.is_alive(), "reader hung")
        stop.set()
        for t in churners:
            t.join(timeout=5.0)
            self.assertFalse(t.is_alive(), "churner hung")

        self.assertEqual(errors, [], f"unexpected errors under churn: {errors!r}")
        with self.web_ui._subscribers_lock:
            # Churners always discard what they admit, so the pool must
            # be empty at the end.
            self.assertEqual(len(pool), 0)

    # ---- WC-D -----------------------------------------------------------
    def test_wc_d_mqtt_fanout_under_churn(self) -> None:
        """`_on_mqtt_message` snapshots the pool and dispatches sends to
        the asyncio loop. While we churn subscribers in/out, the fan-out
        must never raise and at least the steady-state subscribers must
        actually receive messages."""
        cap = self.web_ui.MAX_TELEMETRY_SUBSCRIBERS
        pool = self.web_ui.telemetry_subscribers

        # Steady residents — guaranteed to be in the pool for the whole
        # message stream, so we can assert they actually received frames.
        residents = [FakeWS() for _ in range(2)]
        for w in residents:
            self.assertTrue(self._admit(w, pool, cap, "telemetry"))

        stop = threading.Event()
        errors: list[BaseException] = []

        def churner() -> None:
            try:
                while not stop.is_set():
                    ws = FakeWS()
                    if self._admit(ws, pool, cap, "telemetry"):
                        self._discard(ws, pool)
            except BaseException as exc:
                errors.append(exc)

        class _Msg:
            topic = "lifetrac/v25/telemetry/heartbeat"
            payload = b'{"ok": true}'

        threads = [threading.Thread(target=churner, daemon=True)
                   for _ in range(3)]
        for t in threads:
            t.start()
        try:
            for _ in range(500):
                try:
                    self.web_ui._on_mqtt_message(None, None, _Msg)
                except BaseException as exc:
                    errors.append(exc)
                    break
        finally:
            stop.set()
            for t in threads:
                t.join(timeout=5.0)

        self.assertEqual(errors, [],
                         f"fanout raised under churn: {errors!r}")

        # Drain the loop so all the run_coroutine_threadsafe sends complete
        # before we read counters.
        self.lf.submit(asyncio.sleep(0.05)).result(timeout=2.0)

        for w in residents:
            self.assertGreater(
                w.sent_text, 0,
                "resident subscriber never received a telemetry frame")

        # Resident cleanup so we don't leak into other tests.
        for w in residents:
            self._discard(w, pool)

    # ---- WC-E -----------------------------------------------------------
    def test_wc_e_pool_caps_independent(self) -> None:
        # Saturate telemetry…
        tcap = self.web_ui.MAX_TELEMETRY_SUBSCRIBERS
        for _ in range(tcap):
            self.assertTrue(
                self._admit(FakeWS(), self.web_ui.telemetry_subscribers,
                            tcap, "telemetry"))
        # …and confirm the other three pools still take a fresh client.
        self.assertTrue(self._admit(
            FakeWS(), self.web_ui.image_subscribers,
            self.web_ui.MAX_IMAGE_SUBSCRIBERS, "image"))
        self.assertTrue(self._admit(
            FakeWS(), self.web_ui.state_subscribers,
            self.web_ui.MAX_STATE_SUBSCRIBERS, "state"))
        self.assertTrue(self._admit(
            FakeWS(), self.web_ui.control_subscribers,
            self.web_ui.MAX_CONTROL_SUBSCRIBERS, "control"))

    # ---- WC-F -----------------------------------------------------------
    def test_wc_f_drain_then_refill_no_leak(self) -> None:
        cap = self.web_ui.MAX_IMAGE_SUBSCRIBERS
        pool = self.web_ui.image_subscribers

        # Round 1 — fill to cap, then verify cap+1 is rejected.
        first = [FakeWS() for _ in range(cap)]
        for w in first:
            self.assertTrue(self._admit(w, pool, cap, "image"))
        self.assertFalse(self._admit(FakeWS(), pool, cap, "image"))

        # Drain.
        for w in first:
            self._discard(w, pool)
        with self.web_ui._subscribers_lock:
            self.assertEqual(len(pool), 0)

        # Round 2 — must accept exactly `cap` again, not 0 (would mean a
        # stale counter), not cap+1 (would mean a leaked slot relaxing
        # the bound).
        second = [FakeWS() for _ in range(cap)]
        for w in second:
            self.assertTrue(self._admit(w, pool, cap, "image"))
        self.assertFalse(self._admit(FakeWS(), pool, cap, "image"))


if __name__ == "__main__":                                  # pragma: no cover
    unittest.main()
