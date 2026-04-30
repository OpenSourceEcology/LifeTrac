"""Round 43 / BC-19 — hydraulic build-variant compatibility validator.

The JSON Schema in ``base_station/config/build_config.schema.json`` can
validate each ``[hydraulic]`` leaf in isolation but cannot express the
cross-leaf constraints that the firmware sequencer (BC-18) and the BOM
need:

* ``spool_type ∈ {tandem, closed}`` requires ``load_holding != 'none'``
  (the spool itself is the inherent hold; ``none`` would contradict).
* ``spool_type ∈ {float, open}`` rejects ``load_holding='spool_inherent'``
  (those centres vent cylinder lines on de-energise; the BOM must
  declare PO check, counterbalance, or none).
* ``spool_type ∈ {float, open}`` requires ``valve_settling_ms == 0``
  (no settling delay needed when the centre vents).

These rules live in ``build_config._validate_hydraulic_compatibility``
and run after JSON-Schema validation in ``build_config.load()``.

This SIL gate pins:

* **BC19_A canonical default loads.** Belt-and-braces: the canonical
  TOML (``spool_type='tandem'``, ``load_holding='spool_inherent'``,
  ``valve_settling_ms=100``) is one of the valid combinations.
* **BC19_B every documented reference build loads.** The three
  reference builds enumerated in
  ``DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md`` (OSE-legacy, v25-canonical,
  high-performance) all pass the validator.
* **BC19_C invalid pairings rejected.** Each rule above raises
  ``BuildConfigError`` with a diagnostic that names the offending leaf.
* **BC19_D rejection happens at load() time, not lazily.** A bad config
  fails before any consumer touches it; the consumer never sees a
  partially-validated object.
"""
from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_BASE = _HERE.parent
sys.path.insert(0, str(_BASE))

import build_config  # noqa: E402

_DEFAULT_TOML = _BASE / "config" / "build.default.toml"


def _write_variant(tmpdir: Path, name: str,
                   *, spool: str, hold: str, settle: int) -> Path:
    """Materialise a TOML variant by overriding the three BC-19 leaves."""
    text = _DEFAULT_TOML.read_text(encoding="utf-8")
    replacements = (
        ('spool_type          = "tandem"',          f'spool_type          = "{spool}"'),
        ('load_holding        = "spool_inherent"',  f'load_holding        = "{hold}"'),
        ('valve_settling_ms   = 100',               f'valve_settling_ms   = {settle}'),
    )
    for old, new in replacements:
        if old not in text:
            raise AssertionError(
                f"variant {name!r}: anchor {old!r} missing from default — "
                f"canonical TOML alignment changed?"
            )
        text = text.replace(old, new)
    out = tmpdir / f"build.{name}.toml"
    out.write_text(text, encoding="utf-8")
    return out


def _load(toml_path: Path):
    """Load using the env-var-driven loader. No module reload needed —
    ``build_config.load()`` is purely env-var-driven with no caching.
    Reloading the module would create a fresh ``BuildConfigError`` class
    that ``assertRaises`` (captured pre-reload) would not match."""
    os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(toml_path)
    return build_config.load()


class _Base(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="lt-bc19-"))
        # Clear any leftover env from sibling tests.
        os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)


# --- BC19_A canonical default loads ------------------------------------


class BC19_A_CanonicalLoads(_Base):

    def test_canonical_default_passes_compatibility(self) -> None:
        # The shipped default is one of the valid combinations.
        cfg = _load(_DEFAULT_TOML)
        self.assertEqual(cfg.hydraulic.spool_type, "tandem")
        self.assertEqual(cfg.hydraulic.load_holding, "spool_inherent")
        self.assertEqual(cfg.hydraulic.valve_settling_ms, 100)


# --- BC19_B every documented reference build loads ---------------------


class BC19_B_ReferenceBuildsLoad(_Base):

    def test_ose_legacy(self) -> None:
        # Float spool + PO check + zero settling. Pre-BC-18 interim,
        # documented in DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md.
        path = _write_variant(self.tmp, "ose_legacy",
                              spool="float", hold="po_check", settle=0)
        cfg = _load(path)
        self.assertEqual(cfg.hydraulic.spool_type, "float")
        self.assertEqual(cfg.hydraulic.load_holding, "po_check")
        self.assertEqual(cfg.hydraulic.valve_settling_ms, 0)

    def test_v25_canonical(self) -> None:
        # Tandem spool + inherent hold + 100 ms settle. The canonical
        # v25 build; identical to the default.
        path = _write_variant(self.tmp, "v25_canonical",
                              spool="tandem", hold="spool_inherent", settle=100)
        cfg = _load(path)
        self.assertEqual(cfg.hydraulic.spool_type, "tandem")

    def test_high_performance(self) -> None:
        # Open spool + counterbalance + zero settle. Sport / racing
        # build with free-coast tracks.
        path = _write_variant(self.tmp, "high_perf",
                              spool="open", hold="counterbalance", settle=0)
        cfg = _load(path)
        self.assertEqual(cfg.hydraulic.spool_type, "open")
        self.assertEqual(cfg.hydraulic.load_holding, "counterbalance")

    def test_closed_with_counterbalance(self) -> None:
        # Closed centre + counterbalance + 100 ms settle. Permitted
        # combination (closed has inherent hold but a builder might
        # also want counterbalance for redundancy).
        path = _write_variant(self.tmp, "closed_counter",
                              spool="closed", hold="counterbalance", settle=100)
        cfg = _load(path)
        self.assertEqual(cfg.hydraulic.spool_type, "closed")


# --- BC19_C invalid pairings rejected ----------------------------------


class BC19_C_InvalidPairingsRejected(_Base):

    def test_tandem_with_load_holding_none(self) -> None:
        path = _write_variant(self.tmp, "tandem_none",
                              spool="tandem", hold="none", settle=100)
        with self.assertRaises(build_config.BuildConfigError) as cm:
            _load(path)
        self.assertIn("spool_type='tandem'", str(cm.exception))
        self.assertIn("load_holding", str(cm.exception))

    def test_closed_with_load_holding_none(self) -> None:
        path = _write_variant(self.tmp, "closed_none",
                              spool="closed", hold="none", settle=100)
        with self.assertRaises(build_config.BuildConfigError) as cm:
            _load(path)
        self.assertIn("spool_type='closed'", str(cm.exception))

    def test_float_with_spool_inherent(self) -> None:
        path = _write_variant(self.tmp, "float_inherent",
                              spool="float", hold="spool_inherent", settle=0)
        with self.assertRaises(build_config.BuildConfigError) as cm:
            _load(path)
        self.assertIn("spool_type='float'", str(cm.exception))
        self.assertIn("spool_inherent", str(cm.exception))

    def test_open_with_spool_inherent(self) -> None:
        path = _write_variant(self.tmp, "open_inherent",
                              spool="open", hold="spool_inherent", settle=0)
        with self.assertRaises(build_config.BuildConfigError) as cm:
            _load(path)
        self.assertIn("spool_type='open'", str(cm.exception))

    def test_float_with_nonzero_settling(self) -> None:
        path = _write_variant(self.tmp, "float_settle",
                              spool="float", hold="po_check", settle=100)
        with self.assertRaises(build_config.BuildConfigError) as cm:
            _load(path)
        self.assertIn("spool_type='float'", str(cm.exception))
        self.assertIn("valve_settling_ms=0", str(cm.exception))

    def test_open_with_nonzero_settling(self) -> None:
        path = _write_variant(self.tmp, "open_settle",
                              spool="open", hold="counterbalance", settle=50)
        with self.assertRaises(build_config.BuildConfigError) as cm:
            _load(path)
        self.assertIn("spool_type='open'", str(cm.exception))
        self.assertIn("valve_settling_ms=0", str(cm.exception))


# --- BC19_D rejection happens at load() time ---------------------------


class BC19_D_RejectionAtLoad(_Base):

    def test_invalid_config_does_not_return_partial_object(self) -> None:
        # The validator must raise before BuildConfig() is constructed,
        # so a caller cannot accidentally receive a half-validated
        # object and use it.
        path = _write_variant(self.tmp, "bad",
                              spool="tandem", hold="none", settle=100)
        result = None
        try:
            result = _load(path)
        except build_config.BuildConfigError:
            pass
        self.assertIsNone(result,
                          msg="load() returned a partial object instead of raising")

    def test_source_grep_validator_called_from_load(self) -> None:
        # Tripwire: a refactor that moves _validate_hydraulic_compatibility
        # out of load() (without replacing it) would silently disable the
        # BC-19 gate. Pin the call site by source string.
        text = (Path(build_config.__file__)).read_text(encoding="utf-8")
        self.assertIn("_validate_hydraulic_compatibility(data)", text)
        self.assertIn("def _validate_hydraulic_compatibility", text)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
