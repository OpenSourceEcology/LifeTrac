"""Round 22: Fleet-key provisioning sanity SIL (W4-10 / IP-008).

W4-10 verification surface
==========================

A LifeTrac controller stack must REFUSE TO OPERATE when the AES-128
fleet key is the all-zero placeholder. Three independent layers
enforce this:

1. **Bridge (Linux/X8)** — `lora_bridge._load_fleet_key()` raises
   ``RuntimeError`` on missing / wrong-length / all-zero keys, which
   propagates to a non-zero process exit. Module import is the gate
   so a misconfigured deployment fails in seconds, not at first
   packet.
2. **Tractor M7 (Portenta H7)** — `tractor_h7.ino` `setup()` halts
   in an infinite loop AND writes ``LIFETRAC_ESTOP_MAGIC`` to the
   shared-memory `estop_request` field if `fleet_key_is_zero()` is
   true. The M4 watchdog (already covered in `test_m4_safety_sil.py`)
   then de-energises the safety relay.
3. **Handheld (MKR WAN 1310)** — `handheld_mkr.ino` `setup()` halts
   in an infinite loop AND, if the OLED initialised, prints the
   exact string ``"FLEET KEY NOT\\nPROVISIONED\\nHALT (IP-008)"``
   so a bench operator immediately sees why the unit is unresponsive.

All three guards are wrapped in ``#ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY``
so CI / dev / bench builds can opt out explicitly. The compile gates
themselves (handheld stub-crypto + real-crypto, tractor_h7, opta) DO
define this flag, which is why the all-zero placeholder ships in CI;
production OTA / flash images MUST NOT define it.

What this does NOT cover (bench-only residual)
----------------------------------------------

* Visual confirmation that the OLED renders the text correctly.
* Wall-clock timing of the M4 relay drop after `LIFETRAC_ESTOP_MAGIC`
  is written (covered by `test_m4_safety_sil.py` + W4-03).
* Container exit-code check on a real X8 — the bridge `RuntimeError`
  propagation is verified here, but the systemd unit's restart
  behavior is bench-only.
* Compile-time failure of handheld when `lp_keys_secret.h` is deleted
  (the W4-10 Step-1 procedure) — that is exercised by the existing
  Arduino compile gate failing loud on missing `#include`.
"""

from __future__ import annotations

import importlib
import os
import re
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Iterator

# Repo-relative paths — base_station/tests/ → DESIGN-CONTROLLER/firmware/...
REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_DIR = REPO_ROOT / "DESIGN-CONTROLLER" / "firmware"
TRACTOR_INO = FIRMWARE_DIR / "tractor_h7" / "tractor_h7.ino"
HANDHELD_INO = FIRMWARE_DIR / "handheld_mkr" / "handheld_mkr.ino"
SHARED_MEM_H = FIRMWARE_DIR / "common" / "shared_mem.h"

# Pinned-by-IP-008 — production values these tests defend.
EXPECTED_KEY_LEN_BYTES = 16
EXPECTED_ESTOP_MAGIC = 0xA5A5A5A5
EXPECTED_HANDHELD_OLED_TEXT = "FLEET KEY NOT\\nPROVISIONED\\nHALT (IP-008)"

# Bridge env vars under test.
ENV_FLEET_KEY_FILE = "LIFETRAC_FLEET_KEY_FILE"
ENV_FLEET_KEY_HEX = "LIFETRAC_FLEET_KEY_HEX"
ENV_ALLOW_UNCONFIGURED = "LIFETRAC_ALLOW_UNCONFIGURED_KEY"


# ---------------------------------------------------------------------------
# Helpers — file read + scoped env-var override.
# ---------------------------------------------------------------------------


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


class _EnvOverride:
    """Context manager that temporarily sets / unsets env vars and
    restores them on exit. Used to drive `_load_fleet_key()` through
    its various failure modes."""

    def __init__(self, **overrides: str | None) -> None:
        self._overrides = overrides
        self._saved: dict[str, str | None] = {}

    def __enter__(self) -> "_EnvOverride":
        for k, v in self._overrides.items():
            self._saved[k] = os.environ.get(k)
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
        return self

    def __exit__(self, *exc_info: object) -> None:
        for k, prev in self._saved.items():
            if prev is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = prev


def _load_fleet_key():
    """Import (or re-import) `lora_bridge._load_fleet_key` lazily.

    Importing the module top-level would lock in whichever
    LIFETRAC_ALLOW_UNCONFIGURED_KEY value the test runner had at
    discovery time; we want to control it per-test.
    """
    # Make sure base_station is on sys.path (matches the discover
    # working dir, but be explicit so this file works under -m too).
    bs_dir = str(REPO_ROOT / "DESIGN-CONTROLLER" / "base_station")
    if bs_dir not in sys.path:
        sys.path.insert(0, bs_dir)
    if "lora_bridge" in sys.modules:
        mod = sys.modules["lora_bridge"]
    else:
        mod = importlib.import_module("lora_bridge")
    return mod._load_fleet_key


# ---------------------------------------------------------------------------
# FK-A: shared invariants — placeholder is all-zero, magic value pinned.
# ---------------------------------------------------------------------------


class FK_A_PlaceholderInvariants(unittest.TestCase):
    """The placeholder MUST be all-zero so the runtime check has
    something to detect. The estop magic must be the canonical value
    referenced by the M4 watchdog."""

    _KFLEET_RE = re.compile(
        r"static\s+const\s+uint8_t\s+kFleetKey\s*\[\s*16\s*\]\s*=\s*"
        r"\{\s*(?P<body>[^}]*)\}\s*;",
    )
    _ESTOP_MAGIC_RE = re.compile(
        r"#define\s+LIFETRAC_ESTOP_MAGIC\s+0x([0-9A-Fa-f]+)u?",
    )

    def _assert_kfleet_all_zero(self, source: str, label: str) -> None:
        m = self._KFLEET_RE.search(source)
        self.assertIsNotNone(
            m, f"{label}: kFleetKey[16] declaration not found")
        body = m.group("body").strip()
        # Must be either "0" or "0,0,...,0" — anything else means
        # someone hand-baked a key into source. That is forbidden:
        # production keys live in lp_keys_secret.h (handheld) or
        # come over the secret-mount path (bridge).
        bytes_ = [b.strip() for b in body.split(",") if b.strip()]
        # The conventional "{0}" zero-init form leaves only one element.
        if len(bytes_) == 1:
            self.assertEqual(bytes_[0], "0",
                             f"{label}: kFleetKey single-element init must be {{0}}")
        else:
            self.assertEqual(len(bytes_), EXPECTED_KEY_LEN_BYTES,
                             f"{label}: kFleetKey must be 16-byte init, got {len(bytes_)}")
            for i, b in enumerate(bytes_):
                self.assertEqual(b, "0",
                                 f"{label}: kFleetKey[{i}] must be 0 in source "
                                 f"(production keys belong in lp_keys_secret.h or a "
                                 f"secret mount, not in checked-in firmware)")

    def test_FK_A_handheld_kfleetkey_is_zero_placeholder(self) -> None:
        self._assert_kfleet_all_zero(_read(HANDHELD_INO), "handheld_mkr.ino")

    def test_FK_A2_tractor_kfleetkey_is_zero_placeholder(self) -> None:
        self._assert_kfleet_all_zero(_read(TRACTOR_INO), "tractor_h7.ino")

    def test_FK_A3_estop_magic_constant_pinned(self) -> None:
        # The tractor's IP-008 halt path stores LIFETRAC_ESTOP_MAGIC
        # into shared memory. Make sure the magic value the M4 watchdog
        # looks for hasn't drifted.
        m = self._ESTOP_MAGIC_RE.search(_read(SHARED_MEM_H))
        self.assertIsNotNone(m, "LIFETRAC_ESTOP_MAGIC #define not found")
        self.assertEqual(int(m.group(1), 16), EXPECTED_ESTOP_MAGIC,
                         "LIFETRAC_ESTOP_MAGIC must remain 0xA5A5A5A5 (M4 expects this)")


# ---------------------------------------------------------------------------
# FK-B: handheld provisioning gate.
# ---------------------------------------------------------------------------


class FK_B_HandheldProvisioningGate(unittest.TestCase):
    """The handheld guard must be inside ``#ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY``,
    must call ``fleet_key_is_zero()``, must render the canonical OLED
    string, and must halt in a forever loop so a misprovisioned unit
    cannot transmit."""

    def setUp(self) -> None:
        self.src = _read(HANDHELD_INO)

    def _gate_block(self) -> str:
        # Capture the gate region — from the #ifndef through its #endif.
        m = re.search(
            r"#ifndef\s+LIFETRAC_ALLOW_UNCONFIGURED_KEY\b(?P<body>.*?)#endif",
            self.src,
            re.DOTALL,
        )
        self.assertIsNotNone(
            m, "handheld: #ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY block not found")
        return m.group("body")

    def test_FK_B_fleet_key_zero_check_function_exists(self) -> None:
        self.assertIn("static bool fleet_key_is_zero()", self.src,
                      "handheld must define fleet_key_is_zero() helper")

    def test_FK_B2_gate_calls_fleet_key_is_zero(self) -> None:
        body = self._gate_block()
        self.assertIn("fleet_key_is_zero()", body,
                      "handheld provisioning gate must call fleet_key_is_zero()")

    def test_FK_B3_gate_renders_canonical_oled_text(self) -> None:
        body = self._gate_block()
        # The compile-time literal includes embedded "\n" sequences.
        self.assertIn(EXPECTED_HANDHELD_OLED_TEXT, body,
                      "handheld OLED string drifted from "
                      f"'FLEET KEY NOT\\\\nPROVISIONED\\\\nHALT (IP-008)' — "
                      "operators rely on this exact text on the bench.")

    def test_FK_B4_gate_halts_forever(self) -> None:
        body = self._gate_block()
        # Either `while (1)` or `for (;;)` — both are conventional.
        halts = bool(
            re.search(r"while\s*\(\s*1\s*\)\s*\{[^}]*delay", body)
            or re.search(r"for\s*\(\s*;\s*;\s*\)\s*\{[^}]*delay", body)
        )
        self.assertTrue(
            halts,
            "handheld provisioning gate must halt in a forever-loop with "
            "a delay() inside (otherwise the watchdog might reset and the "
            "unit would silently re-attempt to TX).",
        )

    def test_FK_B5_gate_runs_before_main_tx_loop(self) -> None:
        # The gate MUST be inside setup(), not loop(). Find both
        # function bodies and assert the gate block appears within
        # setup().
        setup_match = re.search(r"\bvoid\s+setup\s*\(\s*\)\s*\{", self.src)
        loop_match = re.search(r"\bvoid\s+loop\s*\(\s*\)\s*\{", self.src)
        self.assertIsNotNone(setup_match, "handheld: setup() not found")
        self.assertIsNotNone(loop_match, "handheld: loop() not found")
        gate_match = re.search(
            r"#ifndef\s+LIFETRAC_ALLOW_UNCONFIGURED_KEY", self.src)
        self.assertIsNotNone(gate_match)
        self.assertGreater(
            gate_match.start(), setup_match.start(),
            "handheld provisioning gate must appear after setup() opens")
        self.assertLess(
            gate_match.start(), loop_match.start(),
            "handheld provisioning gate must run inside setup(), not loop()")


# ---------------------------------------------------------------------------
# FK-C: tractor_h7 provisioning gate.
# ---------------------------------------------------------------------------


class FK_C_TractorProvisioningGate(unittest.TestCase):
    """The tractor M7 guard must be inside ``#ifndef
    LIFETRAC_ALLOW_UNCONFIGURED_KEY``, call ``fleet_key_is_zero()``,
    write ``LIFETRAC_ESTOP_MAGIC`` to ``SHARED->estop_request``
    (so the M4 watchdog drops the safety relay), and halt forever."""

    def setUp(self) -> None:
        self.src = _read(TRACTOR_INO)

    def _gate_block(self) -> str:
        m = re.search(
            r"#ifndef\s+LIFETRAC_ALLOW_UNCONFIGURED_KEY\b(?P<body>.*?)#endif",
            self.src,
            re.DOTALL,
        )
        self.assertIsNotNone(
            m, "tractor_h7: #ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY not found")
        return m.group("body")

    def test_FK_C_fleet_key_zero_check_function_exists(self) -> None:
        self.assertIn("static bool fleet_key_is_zero()", self.src,
                      "tractor must define fleet_key_is_zero() helper")

    def test_FK_C2_gate_calls_fleet_key_is_zero(self) -> None:
        body = self._gate_block()
        self.assertIn("fleet_key_is_zero()", body,
                      "tractor provisioning gate must call fleet_key_is_zero()")

    def test_FK_C3_gate_writes_estop_magic_into_shared_memory(self) -> None:
        body = self._gate_block()
        # The exact assignment that the M4 watchdog poll loop looks for.
        self.assertIn("SHARED->estop_request = LIFETRAC_ESTOP_MAGIC", body,
                      "tractor provisioning gate must set "
                      "SHARED->estop_request = LIFETRAC_ESTOP_MAGIC so the M4 "
                      "watchdog (test_m4_safety_sil.py / W4-03) drops the relay.")

    def test_FK_C4_gate_halts_forever(self) -> None:
        body = self._gate_block()
        halts = bool(
            re.search(r"while\s*\(\s*1\s*\)\s*\{[^}]*delay", body)
            or re.search(r"for\s*\(\s*;\s*;\s*\)\s*\{[^}]*delay", body)
        )
        self.assertTrue(
            halts,
            "tractor provisioning gate must halt in a forever-loop with "
            "delay() inside (mirrors the handheld guard).",
        )


# ---------------------------------------------------------------------------
# FK-D: bridge `_load_fleet_key()` failure modes.
# ---------------------------------------------------------------------------


class FK_D_BridgeLoaderFailures(unittest.TestCase):
    """`lora_bridge._load_fleet_key()` is the Linux-side gate. It must
    raise ``RuntimeError`` (which propagates to a non-zero process
    exit) on missing key, wrong-length key, all-zero key, and bad
    hex. It must succeed only on a valid 16-byte non-zero key."""

    def setUp(self) -> None:
        self.load = _load_fleet_key()

    # ---- happy path -----------------------------------------------

    def test_FK_D_valid_hex_env_loads_clean(self) -> None:
        key_hex = "0102030405060708090a0b0c0d0e0f10"
        with _EnvOverride(**{
            ENV_FLEET_KEY_FILE: None,
            ENV_FLEET_KEY_HEX: key_hex,
            ENV_ALLOW_UNCONFIGURED: None,
        }):
            key = self.load()
        self.assertEqual(key, bytes.fromhex(key_hex))
        self.assertEqual(len(key), EXPECTED_KEY_LEN_BYTES)

    def test_FK_D2_valid_file_raw_bytes_loads_clean(self) -> None:
        key = bytes(range(1, 17))   # 1..16, definitely not all-zero
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as fp:
            fp.write(key)
            path = fp.name
        try:
            with _EnvOverride(**{
                ENV_FLEET_KEY_FILE: path,
                ENV_FLEET_KEY_HEX: None,
                ENV_ALLOW_UNCONFIGURED: None,
            }):
                got = self.load()
            self.assertEqual(got, key)
        finally:
            os.unlink(path)

    def test_FK_D3_valid_file_hex_string_loads_clean(self) -> None:
        key_hex = "112233445566778899aabbccddeeff00"
        with tempfile.NamedTemporaryFile(
                mode="w", suffix=".hex", delete=False, encoding="ascii") as fp:
            fp.write(key_hex)
            path = fp.name
        try:
            with _EnvOverride(**{
                ENV_FLEET_KEY_FILE: path,
                ENV_FLEET_KEY_HEX: None,
                ENV_ALLOW_UNCONFIGURED: None,
            }):
                got = self.load()
            self.assertEqual(got, bytes.fromhex(key_hex))
        finally:
            os.unlink(path)

    # ---- failure paths --------------------------------------------

    def test_FK_D4_missing_env_raises(self) -> None:
        with _EnvOverride(**{
            ENV_FLEET_KEY_FILE: None,
            ENV_FLEET_KEY_HEX: None,
            ENV_ALLOW_UNCONFIGURED: None,
        }):
            with self.assertRaises(RuntimeError) as ctx:
                self.load()
        self.assertIn("not configured", str(ctx.exception).lower())

    def test_FK_D5_all_zero_hex_env_raises(self) -> None:
        with _EnvOverride(**{
            ENV_FLEET_KEY_FILE: None,
            ENV_FLEET_KEY_HEX: "00" * 16,
            ENV_ALLOW_UNCONFIGURED: None,
        }):
            with self.assertRaises(RuntimeError) as ctx:
                self.load()
        msg = str(ctx.exception).lower()
        self.assertIn("all zero", msg,
                      "all-zero key error message must call out 'all zero' so "
                      "the operator immediately knows what's wrong")
        self.assertIn("ip-008", msg,
                      "error must reference IP-008 for runbook traceability")

    def test_FK_D6_all_zero_file_raises(self) -> None:
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as fp:
            fp.write(bytes(EXPECTED_KEY_LEN_BYTES))   # all-zero
            path = fp.name
        try:
            with _EnvOverride(**{
                ENV_FLEET_KEY_FILE: path,
                ENV_FLEET_KEY_HEX: None,
                ENV_ALLOW_UNCONFIGURED: None,
            }):
                with self.assertRaises(RuntimeError) as ctx:
                    self.load()
            self.assertIn("all zero", str(ctx.exception).lower())
        finally:
            os.unlink(path)

    def test_FK_D7_wrong_length_raises(self) -> None:
        # 8 bytes, definitely not 16.
        with _EnvOverride(**{
            ENV_FLEET_KEY_FILE: None,
            ENV_FLEET_KEY_HEX: "0102030405060708",
            ENV_ALLOW_UNCONFIGURED: None,
        }):
            with self.assertRaises(RuntimeError) as ctx:
                self.load()
        self.assertIn("16 bytes", str(ctx.exception),
                      "wrong-length error must state expected length so "
                      "the operator knows the format")

    def test_FK_D8_invalid_hex_raises(self) -> None:
        with _EnvOverride(**{
            ENV_FLEET_KEY_FILE: None,
            ENV_FLEET_KEY_HEX: "ZZ" * 16,
            ENV_ALLOW_UNCONFIGURED: None,
        }):
            with self.assertRaises(RuntimeError) as ctx:
                self.load()
        self.assertIn("hex", str(ctx.exception).lower())

    def test_FK_D9_unreadable_file_raises(self) -> None:
        with _EnvOverride(**{
            ENV_FLEET_KEY_FILE: "/definitely/does/not/exist/fleet.key",
            ENV_FLEET_KEY_HEX: None,
            ENV_ALLOW_UNCONFIGURED: None,
        }):
            with self.assertRaises(RuntimeError) as ctx:
                self.load()
        self.assertIn("unreadable", str(ctx.exception).lower())

    def test_FK_D10_file_with_invalid_hex_chars_raises(self) -> None:
        with tempfile.NamedTemporaryFile(
                mode="w", suffix=".hex", delete=False, encoding="ascii") as fp:
            fp.write("Z" * 32)   # 32 chars but not hex
            path = fp.name
        try:
            with _EnvOverride(**{
                ENV_FLEET_KEY_FILE: path,
                ENV_FLEET_KEY_HEX: None,
                ENV_ALLOW_UNCONFIGURED: None,
            }):
                with self.assertRaises(RuntimeError) as ctx:
                    self.load()
            self.assertIn("hex", str(ctx.exception).lower())
        finally:
            os.unlink(path)


# ---------------------------------------------------------------------------
# FK-E: bypass flag is honored only at module level, not inside the loader.
# ---------------------------------------------------------------------------


class FK_E_BypassFlagSemantics(unittest.TestCase):
    """The ``LIFETRAC_ALLOW_UNCONFIGURED_KEY=1`` bypass MUST live in
    the module-level try/except, NOT inside ``_load_fleet_key()``
    itself. That way explicit calls to the loader (e.g. from a future
    Bridge constructor that re-validates) always fail-closed even if
    the env var is set.
    """

    def test_FK_E_loader_ignores_bypass_flag(self) -> None:
        # With bypass flag set AND no key configured, the loader still
        # raises. The bypass only takes effect at module-import time
        # (covered by FK-E2 below).
        load = _load_fleet_key()
        with _EnvOverride(**{
            ENV_FLEET_KEY_FILE: None,
            ENV_FLEET_KEY_HEX: None,
            ENV_ALLOW_UNCONFIGURED: "1",
        }):
            with self.assertRaises(RuntimeError):
                load()

    def test_FK_E2_module_level_bypass_documented_in_source(self) -> None:
        # Tripwire: confirm the bypass lives where we think it does
        # (module top-level try/except, not inside the loader).
        bridge_path = REPO_ROOT / "DESIGN-CONTROLLER" / "base_station" / "lora_bridge.py"
        src = _read(bridge_path)
        # Find the loader body: from `def _load_fleet_key` up to the
        # next column-0 statement (def/class/try/etc.). Anchored by
        # the explicit return statement at the end of the body.
        loader_match = re.search(
            r"def\s+_load_fleet_key\s*\([^)]*\)\s*->\s*bytes:\s*\n"
            r"(?P<body>(?:[ \t]+[^\n]*\n|\n)+?)"
            r"(?=^\S)",
            src,
            re.DOTALL | re.MULTILINE,
        )
        self.assertIsNotNone(loader_match, "could not locate _load_fleet_key body")
        body = loader_match.group("body")
        self.assertIn("return raw", body,
                      "loader-body capture missed the return statement; "
                      "the regex needs adjusting for the new layout")
        self.assertNotIn(
            ENV_ALLOW_UNCONFIGURED, body,
            "_load_fleet_key() must NOT honor LIFETRAC_ALLOW_UNCONFIGURED_KEY "
            "internally — the bypass belongs in the module-level try/except so "
            "explicit production calls always fail-closed.",
        )
        # And the module-level bypass DOES exist somewhere outside the loader.
        body_outside = src.replace(loader_match.group(0), "")
        self.assertIn(
            ENV_ALLOW_UNCONFIGURED, body_outside,
            "module-level LIFETRAC_ALLOW_UNCONFIGURED_KEY bypass missing — "
            "tests and dev runs would fail to import lora_bridge.",
        )


if __name__ == "__main__":
    unittest.main()
