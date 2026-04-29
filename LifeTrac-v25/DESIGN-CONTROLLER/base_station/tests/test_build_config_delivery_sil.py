"""SIL gate for Round 29 -- IP: BC-10 (config delivery + hot-reload contract).

Pins the schema-side and loader-side half of the BC-10 contract; the actual
delivery CLI / installer daemon and base-UI download route ship in Round 29b
once this contract is stable.

BC10_A_ReloadClassAnnotations
    Every leaf in the schema carries a ``reload_class`` annotation, every
    annotation is in the documented enum, and the schema's
    ``firmware_required`` set is exactly the documented set so a future
    schema edit can't silently demote a firmware-coupled field to
    ``live`` without breaking this test.

BC10_B_ReloadClassDiff
    ``diff_reload_classes`` returns an empty diff for identical configs,
    surfaces every changed leaf path, classifies each by the schema, and
    reports the strictest-class-required as ``worst``. Drift between the
    schema and the data (a flattened key with no schema entry) raises
    rather than defaulting.

BC10_C_Quiescence
    ``evaluate_quiescence`` blocks when the tractor isn't parked long
    enough, when a control session is active, when the M7 TX queue is
    non-empty, and when the engine is under load -- and only returns OK
    when all four conditions hold. Reason strings are operator-facing.

BC10_D_LeafCoverage
    Every leaf the validator iterates has an entry in the reload-class
    map (no schema/loader drift) and the firmware_required set never
    grows silently -- expanding it is a deliberate choice.

BC10_E_SourceTripwire
    The BC-10 helpers are present in build_config.py and the schema
    file by greppable name, so a future refactor can't delete the
    contract.
"""

from __future__ import annotations

import json
import unittest
from pathlib import Path

from build_config import (  # type: ignore[import-not-found]
    RELOAD_CLASSES,
    BuildConfigError,
    QuiescenceState,
    diff_reload_classes,
    evaluate_quiescence,
    iter_reload_classes,
    load,
    load_schema,
)

_BS = Path(__file__).resolve().parents[1]
_SCHEMA_PATH = _BS / "config" / "build_config.schema.json"
_LOADER_PATH = _BS / "build_config.py"

_FIRMWARE_REQUIRED_LEAVES = {"schema_version", "safety.m4_watchdog_ms"}


class BC10_A_ReloadClassAnnotations(unittest.TestCase):
    """Every leaf carries a valid reload_class; the firmware set is fixed."""

    def setUp(self) -> None:
        self.schema = load_schema()
        self.classes = iter_reload_classes(self.schema)

    def test_every_leaf_has_reload_class(self) -> None:
        # iter_reload_classes raises on missing/invalid; just running it
        # to completion and asserting non-empty is the contract.
        self.assertGreater(len(self.classes), 20)
        for path, rc in self.classes.items():
            self.assertIn(rc, RELOAD_CLASSES, msg=f"{path} carries {rc!r}")

    def test_reload_classes_enum_exact(self) -> None:
        self.assertEqual(
            RELOAD_CLASSES, ("live", "restart_required", "firmware_required")
        )

    def test_firmware_required_set_is_pinned(self) -> None:
        actual = {p for p, rc in self.classes.items() if rc == "firmware_required"}
        self.assertEqual(
            actual,
            _FIRMWARE_REQUIRED_LEAVES,
            msg=(
                "firmware_required leaves drifted -- expanding this set means a "
                "re-flash is now needed; demoting means it isn't. Either way, "
                "update _FIRMWARE_REQUIRED_LEAVES deliberately."
            ),
        )

    def test_missing_reload_class_raises(self) -> None:
        bad_schema = {
            "type": "object",
            "additionalProperties": False,
            "properties": {"foo": {"type": "integer"}},
        }
        with self.assertRaises(BuildConfigError):
            iter_reload_classes(bad_schema)

    def test_unknown_reload_class_raises(self) -> None:
        bad_schema = {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "foo": {"type": "integer", "reload_class": "whenever"}
            },
        }
        with self.assertRaises(BuildConfigError):
            iter_reload_classes(bad_schema)


class BC10_B_ReloadClassDiff(unittest.TestCase):
    """diff_reload_classes classifies the change set correctly."""

    def setUp(self) -> None:
        self.base = load()

    def test_identical_configs_empty_diff(self) -> None:
        also = load()
        diff = diff_reload_classes(self.base, also)
        self.assertTrue(diff.is_empty)
        self.assertEqual(diff.changed, ())
        self.assertIsNone(diff.worst)
        self.assertFalse(diff.restart_required)
        self.assertFalse(diff.firmware_required)

    def test_live_only_change(self) -> None:
        # Mutate a `live` leaf (max_control_subscribers) via raw-dict surgery.
        new_raw = json.loads(json.dumps(dict(self.base.raw)))
        new_raw["ui"]["max_control_subscribers"] = (
            self.base.ui.max_control_subscribers + 1
        )
        new = _rebuild(self.base, new_raw)
        diff = diff_reload_classes(self.base, new)
        self.assertEqual(diff.changed, ("ui.max_control_subscribers",))
        self.assertEqual(diff.worst, "live")
        self.assertFalse(diff.restart_required)

    def test_restart_required_change(self) -> None:
        new_raw = json.loads(json.dumps(dict(self.base.raw)))
        new_raw["comm"]["lora_region"] = (
            "eu868" if self.base.comm.lora_region != "eu868" else "us915"
        )
        new = _rebuild(self.base, new_raw)
        diff = diff_reload_classes(self.base, new)
        self.assertEqual(diff.changed, ("comm.lora_region",))
        self.assertEqual(diff.worst, "restart_required")
        self.assertTrue(diff.restart_required)
        self.assertFalse(diff.firmware_required)

    def test_firmware_required_wins(self) -> None:
        new_raw = json.loads(json.dumps(dict(self.base.raw)))
        new_raw["ui"]["max_control_subscribers"] = (
            self.base.ui.max_control_subscribers + 1
        )  # live
        new_raw["comm"]["lora_region"] = (
            "eu868" if self.base.comm.lora_region != "eu868" else "us915"
        )  # restart_required
        new_raw["safety"]["m4_watchdog_ms"] = (
            self.base.safety.m4_watchdog_ms + 50
        )  # firmware_required
        new = _rebuild(self.base, new_raw)
        diff = diff_reload_classes(self.base, new)
        self.assertEqual(set(diff.changed), {
            "ui.max_control_subscribers",
            "comm.lora_region",
            "safety.m4_watchdog_ms",
        })
        self.assertEqual(diff.worst, "firmware_required")
        self.assertTrue(diff.restart_required)
        self.assertTrue(diff.firmware_required)

    def test_undeclared_leaf_in_data_raises(self) -> None:
        # Smuggle a leaf the schema doesn't know about; diff_reload_classes
        # must complain rather than silently accept it.
        new_raw = json.loads(json.dumps(dict(self.base.raw)))
        new_raw["ui"]["surprise_field"] = True
        new = _rebuild(self.base, new_raw, validate=False)
        with self.assertRaises(BuildConfigError):
            diff_reload_classes(self.base, new)


class BC10_C_Quiescence(unittest.TestCase):
    """evaluate_quiescence enforces all four preconditions."""

    def _ok_state(self) -> QuiescenceState:
        return QuiescenceState(
            parked_seconds=60.0,
            active_control_subscribers=0,
            m7_tx_queue_depth=0,
            engine_idle_or_off=True,
        )

    def test_all_conditions_hold(self) -> None:
        result = evaluate_quiescence(self._ok_state())
        self.assertTrue(result.ok)
        self.assertIsNone(result.reason)
        self.assertTrue(bool(result))

    def test_blocks_when_recently_parked(self) -> None:
        state = QuiescenceState(
            parked_seconds=10.0,
            active_control_subscribers=0,
            m7_tx_queue_depth=0,
            engine_idle_or_off=True,
        )
        result = evaluate_quiescence(state)
        self.assertFalse(result.ok)
        self.assertIn("parked", result.reason or "")

    def test_blocks_when_control_session_active(self) -> None:
        state = QuiescenceState(
            parked_seconds=60.0,
            active_control_subscribers=2,
            m7_tx_queue_depth=0,
            engine_idle_or_off=True,
        )
        result = evaluate_quiescence(state)
        self.assertFalse(result.ok)
        self.assertIn("control", result.reason or "")

    def test_blocks_when_radio_busy(self) -> None:
        state = QuiescenceState(
            parked_seconds=60.0,
            active_control_subscribers=0,
            m7_tx_queue_depth=3,
            engine_idle_or_off=True,
        )
        result = evaluate_quiescence(state)
        self.assertFalse(result.ok)
        self.assertIn("radio", result.reason or "")

    def test_blocks_when_engine_under_load(self) -> None:
        state = QuiescenceState(
            parked_seconds=60.0,
            active_control_subscribers=0,
            m7_tx_queue_depth=0,
            engine_idle_or_off=False,
        )
        result = evaluate_quiescence(state)
        self.assertFalse(result.ok)
        self.assertIn("engine", result.reason or "")

    def test_parked_threshold_configurable(self) -> None:
        state = QuiescenceState(
            parked_seconds=15.0,
            active_control_subscribers=0,
            m7_tx_queue_depth=0,
            engine_idle_or_off=True,
        )
        # Default 30s -> blocked
        self.assertFalse(evaluate_quiescence(state).ok)
        # Lower the floor -> passes
        self.assertTrue(evaluate_quiescence(state, parked_seconds_min=10.0).ok)


class BC10_D_LeafCoverage(unittest.TestCase):
    """No drift between the schema's leaf set and the BuildConfig's leaf set."""

    def test_loader_leaves_subset_of_schema_leaves(self) -> None:
        cfg = load()
        from build_config import _flatten  # type: ignore[attr-defined]

        data_leaves = set(_flatten(dict(cfg.raw)).keys())
        schema_leaves = set(iter_reload_classes().keys())
        # Every leaf the loader produced must be classified.
        missing = data_leaves - schema_leaves
        self.assertEqual(missing, set(), msg=f"leaves not in schema: {missing}")
        # And every schema leaf must be present in the canonical data
        # (otherwise the schema declares fields the canonical TOML doesn't carry).
        unused = schema_leaves - data_leaves
        self.assertEqual(unused, set(), msg=f"schema leaves missing from data: {unused}")


class BC10_E_SourceTripwire(unittest.TestCase):
    """BC-10 helpers and schema annotations are greppably present."""

    def test_loader_contains_bc10_helpers(self) -> None:
        text = _LOADER_PATH.read_text(encoding="utf-8")
        for token in (
            "BC-10",
            "iter_reload_classes",
            "diff_reload_classes",
            "evaluate_quiescence",
            "QuiescenceState",
            "ReloadDiff",
        ):
            self.assertIn(token, text, msg=f"loader missing {token!r}")

    def test_schema_uses_reload_class_keyword(self) -> None:
        text = _SCHEMA_PATH.read_text(encoding="utf-8")
        self.assertIn('"reload_class"', text)
        for rc in RELOAD_CLASSES:
            self.assertIn(f'"{rc}"', text, msg=f"schema missing {rc!r}")


# ----------------------------------------------------------------- helpers


def _rebuild(base, new_raw, *, validate: bool = True):
    """Construct a new BuildConfig from a mutated raw dict.

    When ``validate`` is True the new raw is round-tripped through the
    schema validator first so the test exercises a realistic post-load
    state. ``validate=False`` is used to inject a leaf the schema doesn't
    declare, exclusively for the negative-path test in BC10_B.
    """
    from build_config import (  # type: ignore[import-not-found]
        AuxConfig,
        BuildConfig,
        CamerasConfig,
        CommConfig,
        HydraulicConfig,
        NetConfig,
        SafetyConfig,
        SensorsConfig,
        UiConfig,
        _validate,
    )

    if validate:
        _validate(new_raw, load_schema(), path="")
    return BuildConfig(
        unit_id=new_raw["unit_id"],
        schema_version=new_raw["schema_version"],
        hydraulic=HydraulicConfig(**{
            k: v for k, v in new_raw["hydraulic"].items()
            if k in HydraulicConfig.__dataclass_fields__
        }),
        safety=SafetyConfig(**{
            k: v for k, v in new_raw["safety"].items()
            if k in SafetyConfig.__dataclass_fields__
        }),
        cameras=CamerasConfig(**{
            k: v for k, v in new_raw["cameras"].items()
            if k in CamerasConfig.__dataclass_fields__
        }),
        sensors=SensorsConfig(**{
            k: v for k, v in new_raw["sensors"].items()
            if k in SensorsConfig.__dataclass_fields__
        }),
        comm=CommConfig(**{
            k: v for k, v in new_raw["comm"].items()
            if k in CommConfig.__dataclass_fields__
        }),
        ui=UiConfig(**{
            k: v for k, v in new_raw["ui"].items()
            if k in UiConfig.__dataclass_fields__
        }),
        net=NetConfig(**{
            k: v for k, v in new_raw["net"].items()
            if k in NetConfig.__dataclass_fields__
        }),
        aux=AuxConfig(**{
            k: v for k, v in new_raw["aux"].items()
            if k in AuxConfig.__dataclass_fields__
        }),
        source_path=base.source_path,
        raw=new_raw,
    )


if __name__ == "__main__":
    unittest.main()
