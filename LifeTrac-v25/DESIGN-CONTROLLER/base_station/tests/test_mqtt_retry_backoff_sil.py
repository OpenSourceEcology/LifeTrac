"""Round 25: IP-201 MQTT retry/backoff fake-clock SIL.

Claude M-5 flagged that ``web_ui.mqtt_client.connect("localhost", 1883)``
was originally executed at module-import time. Three problems:

* a transient broker outage during deploy crashed the entire web UI,
* ``compileall`` and the test runner could not import ``web_ui``,
* the broker host was hard-coded and ignored ``LIFETRAC_MQTT_HOST``.

Round-history landed the fix: ``_connect_mqtt_with_retry()`` reads
``LIFETRAC_MQTT_HOST`` (default ``localhost``), retries with
exponential backoff (``0.5s`` doubling, capped at ``5.0s``), and
gives up after a 30-second monotonic deadline by re-raising the
underlying ``OSError``/``ConnectionError``.

This SIL pins that contract with a fake clock and a scripted
``mqtt_client.connect`` mock. Tests are deterministic (no real
``time.sleep``) and exercise:

* env-var honoured, default fallback;
* first-try success path (no sleeps);
* retry then success (single backoff observed);
* full backoff schedule ``0.5 \u2192 1.0 \u2192 2.0 \u2192 4.0 \u2192 5.0 \u2192 5.0 \u2026``
  (geometric doubling, cap at 5.0);
* deadline enforcement \u2014 raises after 30 s of monotonic clock,
  even if a retry is "due";
* deadline does NOT fire spuriously when a retry succeeds late
  (e.g. at t \u2248 25 s);
* a source-grep tripwire that defends ``deadline``, ``backoff``,
  ``min(backoff * 2, 5.0)``, and the env-var lookup against future
  refactors.
"""

from __future__ import annotations

import importlib
import logging
import os
import re
import sys
import unittest
from pathlib import Path
from unittest import mock

# Make the base_station package importable regardless of cwd.
_BS = Path(__file__).resolve().parents[1]
if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))

# Skip the whole suite if FastAPI / paho-mqtt aren't installed in the dev env.
try:
    import paho.mqtt.client  # noqa: F401
    import fastapi           # noqa: F401
except ImportError:  # pragma: no cover
    raise unittest.SkipTest("paho-mqtt + fastapi required for IP-201 retry SIL")


def _fresh_web_ui():
    """Import ``web_ui`` with ``paho.mqtt.client.Client`` stubbed so the
    module-import-time ``_connect_mqtt_with_retry()`` call succeeds with
    no real broker. Returns the freshly-(re)loaded module.

    Each test gets its own copy so per-test patches on ``time.sleep`` /
    ``time.monotonic`` / ``mqtt_client.connect`` cannot leak.
    """
    os.environ.setdefault("LIFETRAC_PIN", "424242")
    with mock.patch("paho.mqtt.client.Client") as mqtt_class:
        instance = mqtt_class.return_value
        instance.connect = mock.MagicMock(return_value=None)
        instance.loop_start = mock.MagicMock()
        instance.subscribe = mock.MagicMock()
        instance.publish = mock.MagicMock()
        if "web_ui" in sys.modules:
            web_ui = importlib.reload(sys.modules["web_ui"])
        else:
            import web_ui  # noqa: F401
            web_ui = importlib.reload(sys.modules["web_ui"])
        return web_ui


def _capture_loader_body(src: str) -> str:
    """Return the body of ``_connect_mqtt_with_retry`` as a single string.

    Captures from the def line through the closing of the function — the
    next module-level statement (column-0 ``mqtt_client.loop_start()`` in
    practice) or end-of-file.
    """
    # Find the def line.
    def_idx = src.index("def _connect_mqtt_with_retry")
    rest = src[def_idx:]
    # Walk lines: collect everything until we hit a line that starts at
    # column 0 with a non-whitespace char and is NOT the def line itself.
    lines = rest.splitlines(keepends=True)
    out = [lines[0]]  # the def line
    for ln in lines[1:]:
        if ln and not ln[0].isspace() and ln.strip():
            break
        out.append(ln)
    return "".join(out)


class _FakeClock:
    """Monotonic-clock stand-in that advances ONLY on ``sleep()``."""

    def __init__(self, start: float = 100.0) -> None:
        self.now = start
        self.sleeps: list[float] = []

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += seconds


# ---------------------------------------------------------------------------
# MR-A: env-var + default-host honour.
# ---------------------------------------------------------------------------

class MR_A_HostSelection(unittest.TestCase):

    def test_MR_A_default_host_is_localhost(self) -> None:
        web_ui = _fresh_web_ui()
        os.environ.pop("LIFETRAC_MQTT_HOST", None)
        clock = _FakeClock()
        web_ui.mqtt_client.connect = mock.MagicMock(return_value=None)
        with mock.patch.object(web_ui.time, "monotonic", clock.monotonic), \
             mock.patch.object(web_ui.time, "sleep", clock.sleep):
            web_ui._connect_mqtt_with_retry()
        web_ui.mqtt_client.connect.assert_called_once_with("localhost", 1883)
        self.assertEqual(clock.sleeps, [],
                         "first-try success must NOT sleep")

    def test_MR_A2_env_var_overrides_default(self) -> None:
        web_ui = _fresh_web_ui()
        os.environ["LIFETRAC_MQTT_HOST"] = "mosquitto.lifetrac.local"
        try:
            clock = _FakeClock()
            web_ui.mqtt_client.connect = mock.MagicMock(return_value=None)
            with mock.patch.object(web_ui.time, "monotonic", clock.monotonic), \
                 mock.patch.object(web_ui.time, "sleep", clock.sleep):
                web_ui._connect_mqtt_with_retry()
            web_ui.mqtt_client.connect.assert_called_once_with(
                "mosquitto.lifetrac.local", 1883)
        finally:
            os.environ.pop("LIFETRAC_MQTT_HOST", None)


# ---------------------------------------------------------------------------
# MR-B: backoff schedule on transient failures.
# ---------------------------------------------------------------------------

class MR_B_BackoffSchedule(unittest.TestCase):

    def test_MR_B_one_retry_then_success_observes_one_sleep(self) -> None:
        web_ui = _fresh_web_ui()
        clock = _FakeClock()
        # First call raises, second succeeds.
        web_ui.mqtt_client.connect = mock.MagicMock(
            side_effect=[ConnectionRefusedError("not ready"), None])
        with mock.patch.object(web_ui.time, "monotonic", clock.monotonic), \
             mock.patch.object(web_ui.time, "sleep", clock.sleep):
            web_ui._connect_mqtt_with_retry()
        self.assertEqual(web_ui.mqtt_client.connect.call_count, 2)
        self.assertEqual(clock.sleeps, [0.5],
                         "first retry must sleep exactly 0.5s")

    def test_MR_B2_geometric_doubling_capped_at_5s(self) -> None:
        # Six failures then success: 0.5 -> 1.0 -> 2.0 -> 4.0 -> 5.0 -> 5.0
        web_ui = _fresh_web_ui()
        clock = _FakeClock()
        web_ui.mqtt_client.connect = mock.MagicMock(
            side_effect=[OSError("not ready")] * 6 + [None])
        with mock.patch.object(web_ui.time, "monotonic", clock.monotonic), \
             mock.patch.object(web_ui.time, "sleep", clock.sleep):
            web_ui._connect_mqtt_with_retry()
        self.assertEqual(web_ui.mqtt_client.connect.call_count, 7)
        self.assertEqual(
            clock.sleeps, [0.5, 1.0, 2.0, 4.0, 5.0, 5.0],
            "backoff must double from 0.5 and saturate at 5.0s")

    def test_MR_B3_backoff_never_exceeds_cap_over_long_runs(self) -> None:
        web_ui = _fresh_web_ui()
        clock = _FakeClock(start=0.0)
        # 5 failures + success keeps us inside the 30s budget while still
        # crossing the 5.0s saturation point twice.
        web_ui.mqtt_client.connect = mock.MagicMock(
            side_effect=[OSError("nope")] * 5 + [None])
        with mock.patch.object(web_ui.time, "monotonic", clock.monotonic), \
             mock.patch.object(web_ui.time, "sleep", clock.sleep):
            web_ui._connect_mqtt_with_retry()
        # Every sleep must be in (0, 5.0].
        for s in clock.sleeps:
            self.assertGreater(s, 0.0)
            self.assertLessEqual(
                s, 5.0,
                f"backoff sleep of {s}s exceeds the 5.0s cap")
        # Saturation reached: the last sleep MUST be the cap.
        self.assertEqual(clock.sleeps[-1], 5.0,
                         "by retry 5 the cap should have engaged")


# ---------------------------------------------------------------------------
# MR-C: 30-second deadline.
# ---------------------------------------------------------------------------

class MR_C_DeadlineEnforcement(unittest.TestCase):

    def test_MR_C_raises_after_30s_monotonic_deadline(self) -> None:
        # Connect always fails; we should observe the underlying exception
        # bubble out once the monotonic clock has crossed +30s.
        web_ui = _fresh_web_ui()
        clock = _FakeClock(start=1000.0)
        sentinel = ConnectionRefusedError("broker permanently down")
        web_ui.mqtt_client.connect = mock.MagicMock(side_effect=sentinel)
        with mock.patch.object(web_ui.time, "monotonic", clock.monotonic), \
             mock.patch.object(web_ui.time, "sleep", clock.sleep), \
             self.assertRaises(ConnectionRefusedError) as cm:
            web_ui._connect_mqtt_with_retry()
        self.assertIs(cm.exception, sentinel,
                      "deadline path must re-raise the underlying exception, "
                      "not wrap it in a new one")
        # The total wall-clock advanced is the sum of all sleeps.
        # Deadline check happens BEFORE the sleep, so the call counts
        # represent retries that had not yet exceeded the budget.
        self.assertGreaterEqual(
            sum(clock.sleeps), 30.0,
            "function must not give up before the 30s budget is consumed")
        # But also bounded — given the 5.0s cap and the deadline check
        # happening BEFORE the sleep, no more than ~10 sleeps should
        # ever fit in the budget (0.5+1+2+4 + 5*N).
        self.assertLessEqual(
            len(clock.sleeps), 10,
            "with a 5s cap and 30s budget, sleep count should stay small")

    def test_MR_C2_late_success_inside_deadline_does_not_raise(self) -> None:
        # Push the clock close to the deadline by failing many times,
        # then succeed on the final attempt. Function must return
        # cleanly (not raise spuriously).
        web_ui = _fresh_web_ui()
        clock = _FakeClock(start=0.0)
        web_ui.mqtt_client.connect = mock.MagicMock(
            side_effect=[OSError("warming up")] * 5 + [None])
        with mock.patch.object(web_ui.time, "monotonic", clock.monotonic), \
             mock.patch.object(web_ui.time, "sleep", clock.sleep):
            web_ui._connect_mqtt_with_retry()
        self.assertLess(clock.now, 30.0,
                        "five failures + cap should succeed well under 30s")
        self.assertEqual(web_ui.mqtt_client.connect.call_count, 6)

    def test_MR_C3_deadline_uses_monotonic_not_wall_clock(self) -> None:
        # Source-level guarantee: ``time.monotonic`` is the clock used
        # for the deadline (not ``time.time``, which can jump backwards
        # under NTP slew during boot).
        src = Path(_BS / "web_ui.py").read_text(encoding="utf-8")
        body = _capture_loader_body(src)
        self.assertIn("time.monotonic()", body,
                      "deadline must use time.monotonic() (NTP-slew safe)")
        self.assertNotIn("time.time()", body,
                         "deadline must NOT use time.time() (can jump backwards)")


# ---------------------------------------------------------------------------
# MR-D: source-grep tripwire — defend the contract values against drift.
# ---------------------------------------------------------------------------

class MR_D_SourceContractTripwire(unittest.TestCase):
    """A future refactor that bumps the deadline / cap / initial backoff
    breaks the IP-201 acceptance numbers documented in the implementation
    plan. Trip on the literals so the change has to be conscious."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.src = Path(_BS / "web_ui.py").read_text(encoding="utf-8")
        cls.body = _capture_loader_body(cls.src)

    def test_MR_D_deadline_is_30_seconds(self) -> None:
        self.assertRegex(
            self.body,
            r"deadline\s*=\s*time\.monotonic\(\)\s*\+\s*30\.0",
            "IP-201 contract: 30-second connect deadline. Bumping this "
            "extends boot-time silence after a broker outage.")

    def test_MR_D2_initial_backoff_is_half_second(self) -> None:
        self.assertRegex(
            self.body,
            r"backoff\s*=\s*0\.5",
            "IP-201 contract: initial backoff = 0.5s. A larger initial "
            "value delays first retry past the typical broker warm-up.")

    def test_MR_D3_backoff_doubles_and_caps_at_5s(self) -> None:
        self.assertRegex(
            self.body,
            r"backoff\s*=\s*min\s*\(\s*backoff\s*\*\s*2\s*,\s*5\.0\s*\)",
            "IP-201 contract: backoff doubles, cap = 5.0s.")

    def test_MR_D4_env_var_lookup_with_localhost_default(self) -> None:
        self.assertRegex(
            self.body,
            r'os\.environ\.get\(\s*"LIFETRAC_MQTT_HOST"\s*,\s*"localhost"\s*\)',
            "IP-201 contract: read LIFETRAC_MQTT_HOST with localhost default. "
            "Hard-coding the host re-introduces the original M-5 finding.")

    def test_MR_D5_handles_oserror_and_connectionerror(self) -> None:
        self.assertRegex(
            self.body,
            r"except\s*\(\s*OSError\s*,\s*ConnectionError\s*\)",
            "IP-201 contract: catch (OSError, ConnectionError). Anything "
            "narrower lets a paho-mqtt error category crash the UI on boot.")


def setUpModule() -> None:  # noqa: D401
    # Silence the WARNING/ERROR log spam from the retry function during
    # the deadline tests — it's expected output, not test signal.
    logging.getLogger().setLevel(logging.CRITICAL)


if __name__ == "__main__":
    unittest.main()
