"""Round 31 / BC-03: emit a C header from the canonical build config.

Single source of truth for the firmware sketches. Walks the validated
:class:`build_config.BuildConfig` and emits a deterministic C header
(``lifetrac_build_config.h``) that the M4/M7 sketches can ``#include``
in place of the hand-edited ``#define`` blocks they carry today
(e.g. ``LIFETRAC_M4_WATCHDOG_MS`` in ``firmware/common/shared_mem.h``).

The codegen guarantees:

* **Determinism.** Same input ``BuildConfig`` produces byte-identical
  header output. Sections, leaves, and enum side-macros are emitted in
  a fixed canonical order driven by the dataclass field order. No
  timestamps are baked into the body -- only the source path, unit_id,
  and ``config_sha256`` appear in the leading comment block, all of
  which are stable for a given config.

* **Leaf parity.** Every scalar leaf the JSON Schema declares as a
  property of every section gets a ``#define``. The SIL gate
  (``test_build_config_codegen_sil.py``) walks the schema and asserts
  every leaf appears in the emitted header, so a future schema
  addition that the codegen forgets to handle fails CI.

* **Type-safe macros.**
    - ``int``  -> bare integer literal (e.g. ``2``).
    - ``bool`` -> ``1`` / ``0``.
    - ``float`` -> decimal literal with a trailing ``f``
      (e.g. ``2.5f``) so the firmware compiler picks ``float`` not
      ``double`` and gcc -Wdouble-promotion stays clean.
    - ``str`` (free-form) -> double-quoted C string literal.
    - ``str`` (enum-typed) -> double-quoted string AND a side macro
      ``LIFETRAC_<SECTION>_<NAME>_<UPPER_VALUE>`` defined to ``1`` so
      sketches can use ``#if LIFETRAC_SAFETY_ESTOP_TOPOLOGY_PSR_MONITORED_DUAL``
      preprocessor branches against the canonical truth.

* **Backwards-compat anchors.** A small set of legacy macro names that
  the existing firmware uses are emitted as additional aliases so a
  drop-in replacement is possible without touching every ``.cpp``:
  - ``LIFETRAC_M4_WATCHDOG_MS`` -> alias of ``LIFETRAC_SAFETY_M4_WATCHDOG_MS``.

The module is pure-Python stdlib + ``build_config`` (no third-party
deps); the generated header is plain ASCII so it can be diffed in a
PR without any tooling.
"""

from __future__ import annotations

import dataclasses
from typing import Any, Iterable, Mapping

from build_config import (
    BuildConfig,
    BuildConfigError,
    load_schema,
)

GENERATOR = "lifetrac-config codegen (Round 31 / BC-03)"
HEADER_GUARD = "LIFETRAC_BUILD_CONFIG_H"
MACRO_PREFIX = "LIFETRAC"

# Section name on the dataclass -> TOML / schema section name. We keep
# the dataclass attribute order (Python dict insertion) as the canonical
# emission order; this is the same order as the schema's ``properties``
# block, which is what the loader walks.
_SECTIONS: tuple[str, ...] = (
    "hydraulic",
    "safety",
    "cameras",
    "sensors",
    "comm",
    "ui",
    "net",
    "aux",
)

# Legacy aliases (existing firmware names -> canonical generated name).
# Keep this list short -- prefer migrating sketches to the canonical
# names. Anything in this table is also covered by the SIL gate.
_LEGACY_ALIASES: tuple[tuple[str, str], ...] = (
    ("LIFETRAC_M4_WATCHDOG_MS", "LIFETRAC_SAFETY_M4_WATCHDOG_MS"),
)


def _section_schema(schema: Mapping[str, Any], section: str) -> Mapping[str, Any]:
    """Return the JSON-Schema sub-object for the named section."""
    try:
        return schema["properties"][section]
    except KeyError as exc:  # pragma: no cover -- defensive
        raise BuildConfigError(f"schema missing section {section!r}") from exc


def _macro(section: str, leaf: str) -> str:
    return f"{MACRO_PREFIX}_{section.upper()}_{leaf.upper()}"


def _enum_side_macro(section: str, leaf: str, value: str) -> str:
    safe = value.upper().replace("-", "_").replace(".", "_")
    return f"{MACRO_PREFIX}_{section.upper()}_{leaf.upper()}_{safe}"


def _format_value(value: Any) -> str:
    """Render a scalar Python value as a C literal."""
    # bool first -- isinstance(True, int) is True.
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        # ensure a decimal point so the C compiler doesn't see an int
        text = repr(value)
        if "." not in text and "e" not in text and "E" not in text:
            text = text + ".0"
        return text + "f"
    if isinstance(value, str):
        # Plain ASCII canonical -- the schema's pattern constraints
        # already keep these alphanumeric+dash. Refuse anything that
        # would need escaping rather than silently mis-emit.
        if any(ord(ch) > 0x7E or ord(ch) < 0x20 or ch in {'"', '\\'} for ch in value):
            raise BuildConfigError(
                f"codegen: refusing to emit string with special chars: {value!r}"
            )
        return f'"{value}"'
    raise BuildConfigError(f"codegen: unsupported value type {type(value).__name__}")


def _iter_leaves(cfg: BuildConfig, section: str) -> Iterable[tuple[str, Any]]:
    """Yield (leaf_name, value) for each scalar field on a section."""
    sub = getattr(cfg, section)
    for f in dataclasses.fields(sub):
        yield f.name, getattr(sub, f.name)


def _emit_section(lines: list[str], cfg: BuildConfig,
                  schema: Mapping[str, Any], section: str) -> None:
    section_schema = _section_schema(schema, section)
    properties = section_schema.get("properties", {})
    lines.append("")
    lines.append(f"/* [{section}] */")
    for leaf, value in _iter_leaves(cfg, section):
        macro = _macro(section, leaf)
        lines.append(f"#define {macro} {_format_value(value)}")
        # Enum side macro: only when the schema constrains the leaf to
        # a finite enum set AND the value is a string. We emit a 1
        # macro for the active value so sketches can ``#if`` on it.
        leaf_schema = properties.get(leaf, {})
        enum = leaf_schema.get("enum")
        if isinstance(value, str) and enum:
            for option in enum:
                side = _enum_side_macro(section, leaf, option)
                lines.append(f"#define {side} {1 if option == value else 0}")


def emit_header(cfg: BuildConfig, *, generator: str = GENERATOR) -> str:
    """Emit a deterministic C header capturing every scalar leaf of ``cfg``.

    Output is plain ASCII, LF-terminated, with no trailing whitespace.
    """
    schema = load_schema()
    sha = cfg.config_sha256
    lines: list[str] = []
    lines.append(f"/* AUTOGENERATED by {generator} -- DO NOT EDIT BY HAND. */")
    lines.append("/* Regenerate with:  python tools/lifetrac_config.py codegen --out <path> */")
    # Source filename only (no directory) so the emission is stable
    # across working directories -- a CLI invocation from the repo
    # root and a Python call from anywhere produce byte-identical
    # output for the same TOML file.
    lines.append(f"/* Source:  {cfg.source_path.name} */")
    lines.append(f"/* unit_id: {cfg.unit_id}  schema_version: {cfg.schema_version}  sha256: {sha} */")
    lines.append("")
    lines.append(f"#ifndef {HEADER_GUARD}")
    lines.append(f"#define {HEADER_GUARD}")
    lines.append("")
    lines.append("/* --- identity --- */")
    lines.append(f'#define {MACRO_PREFIX}_UNIT_ID "{cfg.unit_id}"')
    lines.append(f"#define {MACRO_PREFIX}_SCHEMA_VERSION {cfg.schema_version}")
    lines.append(f'#define {MACRO_PREFIX}_CONFIG_SHA256_HEX "{sha}"')
    lines.append(f'#define {MACRO_PREFIX}_CONFIG_SHA256_HEX_SHORT "{sha[:8]}"')
    for section in _SECTIONS:
        _emit_section(lines, cfg, schema, section)
    lines.append("")
    lines.append("/* --- legacy aliases (Round 31 / BC-03 migration) --- */")
    for legacy, canonical in _LEGACY_ALIASES:
        lines.append(f"#define {legacy} {canonical}")
    lines.append("")
    lines.append(f"#endif /* {HEADER_GUARD} */")
    lines.append("")  # trailing newline
    return "\n".join(lines)


def write_header(cfg: BuildConfig, dest, *, generator: str = GENERATOR) -> str:
    """Atomically write the emitted header to ``dest`` (Path or str).

    Returns the rendered header text. Uses the same tempfile + os.replace
    pattern as :func:`installer_daemon._atomic_write` so a partial write
    can never leave a half-rewritten header for the firmware build.
    """
    import os
    import tempfile
    from pathlib import Path

    text = emit_header(cfg, generator=generator)
    target = Path(dest)
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".lifetrac-codegen-", suffix=".h",
                               dir=str(target.parent))
    try:
        with os.fdopen(fd, "w", encoding="ascii", newline="\n") as f:
            f.write(text)
        os.replace(tmp, target)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return text


__all__ = (
    "GENERATOR",
    "HEADER_GUARD",
    "MACRO_PREFIX",
    "emit_header",
    "write_header",
)
