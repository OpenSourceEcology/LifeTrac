#!/usr/bin/env python3
"""``lifetrac-config`` -- offline build-config tooling for laptop / base use.

Round 29b (BC-10 delivery): five subcommands. Round 29b-alpha shipped
``validate`` / ``bundle`` / ``verify`` / ``diff`` (laptop-side, base
side, no tractor I/O). Round 29b-beta adds ``push`` (hand a bundle to
the X8-side installer in :mod:`installer_daemon`) and pairs it with
the X8-side feedback contract (:mod:`feedback`) and the watch-and-
reload helper (:mod:`config_watcher`). Round 31 (BC-03) adds
``codegen`` (emit the canonical C header for the firmware build,
``--check`` mode for CI drift detection). Round 34 (BC-06) adds
``dump-json`` (print the validated config as canonical JSON to
stdout) so non-Python tools — notably ``hil/dispatch.ps1`` —
can apply gate-applicability rules without re-implementing TOML +
schema-validation in PowerShell. Round 37 (BC-14) adds
``inventory`` (aggregate ``config_loaded`` audit lines across one
or more ``audit.jsonl`` files into a fleet-wide CSV / Markdown
table) so a depot operator can answer "which fleet vehicles are
running which build right now?" without writing ad-hoc jq.

Subcommands
-----------

``validate <toml-path>``
    Schema-validate a candidate ``build.<unit>.toml``. Pass / fail with
    a non-zero exit code; intended for CI gates and pre-commit hooks.

``bundle <toml-path> -o <out-dir> [--unit-id <override>]``
    Schema-validate, then write the installer bundle
    ``<out-dir>/lifetrac-config-<unit_id>-<sha8>.toml``. Operator copies
    that single file to a USB stick, walks it to the tractor.

``verify <bundle-path>``
    Re-parse + re-hash a bundle that already exists on disk, and check
    the embedded ``unit_id`` / SHA against the filename. Schema-validates
    the body too. Use after copying to a USB stick on a flaky filesystem.

``diff <toml-or-bundle> --against <toml-or-bundle>``
    Show which leaves differ between two configs and what reload class
    each change requires. Built on the BC-10 ``diff_reload_classes``
    helper from Round 29.

Exits with status 0 on success, 2 on user-facing validation / verification
failure, and reserves 1 for unexpected exceptions (so CI can distinguish
"the config is bad" from "the tool is bad").

Usage from the repo:

.. code-block:: console

    $ python tools/lifetrac-config validate path/to/build.toml
    $ python tools/lifetrac-config bundle path/to/build.toml -o ./out
    $ python tools/lifetrac-config verify ./out/lifetrac-config-foo-12345678.toml
    $ python tools/lifetrac-config diff ./old.toml --against ./new.toml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Make sibling packages importable when invoked as ``python tools/lifetrac-config``.
_ROOT = Path(__file__).resolve().parent.parent
_BASE_STATION = _ROOT / "DESIGN-CONTROLLER" / "base_station"
if str(_BASE_STATION) not in sys.path:
    sys.path.insert(0, str(_BASE_STATION))

import build_config  # noqa: E402  -- after sys.path tweak
import config_bundle  # noqa: E402

EXIT_OK = 0
EXIT_USAGE = 2
TOOL_VERSION = "0.1.0"


def _load_validated(path: Path) -> tuple[build_config.BuildConfig, str]:
    """Load + schema-validate a TOML or bundle file. Returns (cfg, body_text).

    For a bundle, returns the unwrapped body. For a raw TOML, returns the
    file's text verbatim. Either way the caller has a validated
    ``BuildConfig`` and the canonical text it came from.
    """
    text = path.read_text(encoding="utf-8")
    if config_bundle.BODY_SENTINEL in text:
        bundle = config_bundle.parse(text)
        body = bundle.body
        # Round-trip through the loader by setting the env override.
        # We avoid mutating os.environ globally -- write to a temp file
        # the loader can resolve.
        import os
        import tempfile

        with tempfile.NamedTemporaryFile(
            "w", suffix=".toml", delete=False, encoding="utf-8"
        ) as tf:
            tf.write(body)
            tmp_path = Path(tf.name)
        prev = os.environ.get("LIFETRAC_BUILD_CONFIG_PATH")
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(tmp_path)
        try:
            cfg = build_config.load()
        finally:
            if prev is None:
                os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
            else:
                os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = prev
            tmp_path.unlink(missing_ok=True)
        if cfg.unit_id != bundle.unit_id:
            raise config_bundle.BundleError(
                f"bundle header unit_id {bundle.unit_id!r} != body unit_id {cfg.unit_id!r}"
            )
        return cfg, body
    # Plain TOML path.
    import os

    prev = os.environ.get("LIFETRAC_BUILD_CONFIG_PATH")
    os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
    try:
        cfg = build_config.load()
    finally:
        if prev is None:
            os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        else:
            os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = prev
    return cfg, text


def cmd_validate(args: argparse.Namespace) -> int:
    path = Path(args.toml_path)
    cfg, _ = _load_validated(path)
    print(f"OK  {path}")
    print(f"    unit_id:       {cfg.unit_id}")
    print(f"    schema:        v{cfg.schema_version}")
    print(f"    config_sha256: {cfg.config_sha256}")
    return EXIT_OK


def cmd_bundle(args: argparse.Namespace) -> int:
    src = Path(args.toml_path)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg, body = _load_validated(src)
    unit_id = args.unit_id or cfg.unit_id
    if unit_id != cfg.unit_id:
        # Allow override only when the file's own unit_id matches; otherwise
        # someone is deliberately mis-targeting and we refuse.
        raise config_bundle.BundleError(
            f"--unit-id {unit_id!r} != TOML unit_id {cfg.unit_id!r}"
        )
    bundle = config_bundle.make_bundle(
        body, unit_id, generator=f"lifetrac-config {TOOL_VERSION}"
    )
    out_path = out_dir / bundle.filename
    out_path.write_text(config_bundle.serialise(bundle), encoding="utf-8")
    print(f"OK  wrote {out_path}")
    print(f"    unit_id: {bundle.unit_id}")
    print(f"    sha256:  {bundle.sha256}")
    return EXIT_OK


def cmd_verify(args: argparse.Namespace) -> int:
    path = Path(args.bundle_path)
    text = path.read_text(encoding="utf-8")
    bundle = config_bundle.parse(text)  # body-SHA + envelope checks
    config_bundle.verify_filename_matches(bundle, path.name)
    cfg, _ = _load_validated(path)  # schema validation (re-parses bundle)
    print(f"OK  {path}")
    print(f"    unit_id:       {bundle.unit_id}")
    print(f"    sha256:        {bundle.sha256}")
    print(f"    schema:        v{cfg.schema_version}")
    print(f"    generator:     {bundle.generator}")
    print(f"    created:       {bundle.created}")
    return EXIT_OK


def cmd_push(args: argparse.Namespace) -> int:
    """Hand a bundle to the X8-side installer.

    Two modes:

    ``--via local`` (default)
        Copies the bundle file into ``--dest`` (typically a USB-stick
        mount the X8 also watches, or the local inbox path
        ``/var/lib/lifetrac/inbox/``). When ``--apply`` is also given,
        invokes :func:`installer_daemon.process_mount` on the dest
        directory immediately afterwards and prints each
        :class:`installer_daemon.InstallResult` -- this is the SIL
        path and the path used by Round 29b-beta CI.

    ``--via ssh``
        Builds (and prints) an ``scp`` + ``ssh`` command pair that
        copies the bundle onto a remote X8 and triggers the local
        installer there. ``--user`` and ``--dest`` configure the
        remote endpoint. Without ``--execute``, only the planned
        command is printed (this is deliberate -- shelling out to
        SSH from CI is a bench operation, not a unit-test one).
    """
    src = Path(args.bundle_path)
    text = src.read_text(encoding="utf-8")
    bundle = config_bundle.parse(text)
    config_bundle.verify_filename_matches(bundle, src.name)

    if args.via == "local":
        import shutil

        dest_dir = Path(args.dest)
        dest_dir.mkdir(parents=True, exist_ok=True)
        dst = dest_dir / src.name
        shutil.copyfile(src, dst)
        print(f"OK  copied {src} -> {dst}")
        if args.apply:
            if not args.target:
                print(
                    "ERROR  --apply requires --target <active-toml-path>",
                    file=sys.stderr,
                )
                return EXIT_USAGE
            import installer_daemon  # noqa: WPS433  -- lazy: only on --apply

            target = Path(args.target)
            results = installer_daemon.process_mount(
                dest_dir,
                target_path=target,
                unit_id=bundle.unit_id,
                current_cfg=None,
            )
            any_failed = False
            for r in results:
                print(
                    f"    {r.status:<9} {r.unit_id} sha8={(r.applied_sha or '--------')[:8]}"
                    f" reload={r.reload_class}  reason={r.reason}"
                )
                if r.status == "rejected":
                    any_failed = True
            return EXIT_USAGE if any_failed else EXIT_OK
        return EXIT_OK

    # ssh path -- build the command line; only execute on --execute.
    if not args.host:
        print("ERROR  --via ssh requires --host", file=sys.stderr)
        return EXIT_USAGE
    user = args.user or "lifetrac"
    dest_dir = args.dest or "/var/lib/lifetrac/inbox"
    remote = f"{user}@{args.host}"
    scp_cmd = ["scp", str(src), f"{remote}:{dest_dir}/{src.name}"]
    ssh_cmd = [
        "ssh",
        remote,
        f"sudo systemctl start lifetrac-config-installer@{bundle.unit_id}.service",
    ]
    print("PLAN  " + " ".join(scp_cmd))
    print("PLAN  " + " ".join(ssh_cmd))
    if not args.execute:
        print("(--execute not given; nothing actually run)")
        return EXIT_OK
    import subprocess  # noqa: WPS433  -- lazy: only on --execute

    rc = subprocess.run(scp_cmd, check=False).returncode
    if rc != 0:
        print(f"ERROR  scp failed (rc={rc})", file=sys.stderr)
        return EXIT_USAGE
    rc = subprocess.run(ssh_cmd, check=False).returncode
    if rc != 0:
        print(f"ERROR  ssh trigger failed (rc={rc})", file=sys.stderr)
        return EXIT_USAGE
    return EXIT_OK


def cmd_diff(args: argparse.Namespace) -> int:
    old_cfg, _ = _load_validated(Path(args.old_path))
    new_cfg, _ = _load_validated(Path(args.against))
    diff = build_config.diff_reload_classes(old_cfg, new_cfg)
    if diff.is_empty:
        print("OK  configs identical (no diff)")
        return EXIT_OK
    width = max(len(p) for p in diff.changed)
    for path in diff.changed:
        rc = diff.classes[path]
        print(f"  {path:<{width}}  {rc}")
    print(f"--> reload required: {diff.worst}")
    return EXIT_OK


def cmd_codegen(args: argparse.Namespace) -> int:
    """Round 31 / BC-03: emit the canonical C header for the firmware build.

    Reads a TOML or bundle, schema-validates it (loader path), and
    writes a deterministic ``lifetrac_build_config.h`` to ``--out``.
    With ``--check`` the file is not written; instead the rendered text
    is compared against the on-disk file and a non-zero exit code is
    returned on drift (CI gate).
    """
    import build_config_codegen
    cfg, _ = _load_validated(Path(args.toml_path))
    text = build_config_codegen.emit_header(cfg)
    out = Path(args.out)
    if args.check:
        if not out.is_file():
            print(f"ERROR  --check: {out} does not exist", file=sys.stderr)
            return EXIT_USAGE
        on_disk = out.read_text(encoding="ascii")
        if on_disk != text:
            print(f"ERROR  --check: {out} is stale (rerun without --check)",
                  file=sys.stderr)
            return EXIT_USAGE
        print(f"OK  {out} matches canonical codegen output")
        return EXIT_OK
    build_config_codegen.write_header(cfg, out)
    print(f"OK  wrote {out} (sha256={cfg.config_sha256[:8]})")
    return EXIT_OK


def cmd_dump_json(args: argparse.Namespace) -> int:
    """Round 34 / BC-06: print the validated config as canonical JSON to stdout.

    The output is the loader's ``cfg.raw`` round-tripped through
    ``json.dumps(..., sort_keys=True, separators=(',', ':'))`` so the
    same input TOML always produces byte-identical JSON. Consumers
    (notably ``DESIGN-CONTROLLER/hil/dispatch.ps1``) parse this JSON to
    apply per-gate applicability rules without re-implementing TOML
    parsing or schema validation in PowerShell.
    """
    import json
    cfg, _ = _load_validated(Path(args.toml_path))
    payload = {
        "unit_id": cfg.unit_id,
        "schema_version": cfg.schema_version,
        "config_sha256": cfg.config_sha256,
        "config": cfg.raw,
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    print(text)
    return EXIT_OK


# --- Round 41 / BC-12C: dry-run boot self-test ---------------------------


def cmd_self_test(args: argparse.Namespace) -> int:
    """Round 41: run :func:`boot_self_test.run_self_test` against a TOML
    config and a JSON-encoded :class:`HardwareInventory`.

    Designed for two workflows:

    1. Pre-deployment dry-run on the install bench: operator captures
       what the M4 reports as a JSON file, points this command at it
       plus the candidate TOML, and gets a structured pass/fail report
       *before* flashing. No tractor I/O.
    2. Post-incident replay: given a copy of the production
       ``audit.jsonl`` plus the TOML that was active, reproduce the
       same self-test the tractor ran at boot.

    The ``HardwareInventory`` JSON shape mirrors the dataclass field
    names exactly. Exit code is 0 if ``ok`` (no error findings),
    EXIT_USAGE (2) if ``ok=False`` or the inventory file is malformed.
    """
    import json
    import boot_self_test as bst
    cfg, _ = _load_validated(Path(args.toml_path))
    inv_path = Path(args.inventory_path)
    if not inv_path.is_file():
        print(f"ERROR  inventory file not found: {inv_path}", file=sys.stderr)
        return EXIT_USAGE
    try:
        raw = json.loads(inv_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        print(f"ERROR  cannot parse inventory JSON: {exc}", file=sys.stderr)
        return EXIT_USAGE
    if not isinstance(raw, dict):
        print(f"ERROR  inventory JSON must be a top-level object, got {type(raw).__name__}",
              file=sys.stderr)
        return EXIT_USAGE
    try:
        inventory = bst.HardwareInventory(**raw)
    except TypeError as exc:
        print(f"ERROR  inventory shape mismatch: {exc}", file=sys.stderr)
        return EXIT_USAGE
    report = bst.run_self_test(cfg, inventory)
    if args.format == "json":
        payload = {
            "unit_id": report.unit_id,
            "config_sha256": report.config_sha256,
            "ok": report.ok,
            "error_count": len(report.errors()),
            "warning_count": len(report.warnings()),
            "findings": [
                {
                    "severity": f.severity,
                    "code": f.code,
                    "message": f.message,
                    "expected": f.expected,
                    "observed": f.observed,
                }
                for f in report.findings
            ],
        }
        sys.stdout.write(json.dumps(payload, sort_keys=True,
                                    separators=(",", ":")) + "\n")
    else:
        sys.stdout.write(
            f"unit_id        : {report.unit_id}\n"
            f"config_sha256  : {report.config_sha256[:16]}\n"
            f"ok             : {report.ok}\n"
            f"errors         : {len(report.errors())}\n"
            f"warnings       : {len(report.warnings())}\n"
        )
        if report.findings:
            sys.stdout.write("findings:\n")
            for f in report.findings:
                sys.stdout.write(
                    f"  [{f.severity:7s}] {f.code:32s} "
                    f"expected={f.expected!r}  observed={f.observed!r}\n"
                    f"             {f.message}\n"
                )
    return EXIT_OK if report.ok else EXIT_USAGE


# --- Round 37 / BC-14: fleet-wide config inventory ----------------------

# Public so the SIL gate can import + exercise the parser without
# subprocessing the CLI.
INVENTORY_EVENT = "config_loaded"
INVENTORY_FIELDS = (
    "unit_id",
    "config_sha256",
    "schema_version",
    "first_seen",
    "last_seen",
    "boot_count",
    "components",
    "source_paths",
)


def iter_config_loaded_events(paths):
    """Yield decoded ``config_loaded`` records from one or more JSONL files.

    Robust to mixed-event log files (only ``config_loaded`` lines are
    returned), to malformed lines (silently skipped, never raises), and
    to missing files (silently skipped). Records missing the canonical
    set of fields are also skipped \u2014 a quietly partial inventory is
    far less harmful than an exception that aborts the whole report.
    """
    import json
    for path in paths:
        try:
            fp = open(path, "r", encoding="utf-8")
        except OSError:
            continue
        with fp:
            for line in fp:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except (TypeError, ValueError):
                    continue
                if not isinstance(rec, dict):
                    continue
                if rec.get("event") != INVENTORY_EVENT:
                    continue
                # Canonical fields the audit-log emitter writes (see
                # ``web_ui._audit_config_loaded`` and
                # ``lora_bridge.Bridge.__init__``):
                if not all(k in rec for k in ("ts", "unit_id",
                                              "config_sha256")):
                    continue
                yield rec


def aggregate_inventory(records):
    """Fold an iterable of ``config_loaded`` records into one row per
    (unit_id, config_sha256) pair.

    The aggregation key is deliberately *both* unit_id and SHA so a
    single fleet vehicle that has been reflashed across two distinct
    builds produces two rows \u2014 a depot operator wants to see that
    the unit ran build A from t0..t1 and build B from t2..t3, not a
    single row that elides the transition.
    """
    rows: "dict[tuple[str, str], dict]" = {}
    for rec in records:
        key = (rec["unit_id"], rec["config_sha256"])
        ts = float(rec["ts"])
        component = str(rec.get("component", ""))
        source = str(rec.get("source_path", ""))
        schema_version = rec.get("schema_version")
        cur = rows.get(key)
        if cur is None:
            rows[key] = {
                "unit_id": rec["unit_id"],
                "config_sha256": rec["config_sha256"],
                "schema_version": schema_version,
                "first_seen": ts,
                "last_seen": ts,
                "boot_count": 1,
                "components": {component} if component else set(),
                "source_paths": {source} if source else set(),
            }
            continue
        cur["boot_count"] += 1
        if ts < cur["first_seen"]:
            cur["first_seen"] = ts
        if ts > cur["last_seen"]:
            cur["last_seen"] = ts
        if component:
            cur["components"].add(component)
        if source:
            cur["source_paths"].add(source)
        # Schema_version may be absent on older records; keep the latest
        # observed non-None value.
        if schema_version is not None:
            cur["schema_version"] = schema_version
    # Stable sort: unit_id, then last_seen descending (most-recent build
    # for a unit floats to the top within its unit-group).
    return sorted(
        rows.values(),
        key=lambda r: (r["unit_id"], -r["last_seen"]),
    )


def _format_ts(ts: float) -> str:
    """ISO-8601 UTC, second precision. Empty string for zero / negative."""
    import datetime
    if not ts or ts <= 0:
        return ""
    return datetime.datetime.fromtimestamp(
        ts, tz=datetime.timezone.utc
    ).strftime("%Y-%m-%dT%H:%M:%SZ")


def render_inventory_csv(rows) -> str:
    """Render aggregated rows as RFC-4180 CSV (LF newlines).

    Multi-valued fields (``components``, ``source_paths``) are joined
    with ``;`` because a CSV cell cannot legally contain a comma without
    quoting and ``;`` is the conventional secondary delimiter.
    """
    import csv
    import io
    buf = io.StringIO()
    writer = csv.writer(buf, lineterminator="\n")
    writer.writerow(INVENTORY_FIELDS)
    for r in rows:
        writer.writerow([
            r["unit_id"],
            r["config_sha256"][:16],
            r["schema_version"] if r["schema_version"] is not None else "",
            _format_ts(r["first_seen"]),
            _format_ts(r["last_seen"]),
            r["boot_count"],
            ";".join(sorted(r["components"])),
            ";".join(sorted(r["source_paths"])),
        ])
    return buf.getvalue()


def render_inventory_markdown(rows) -> str:
    """Render aggregated rows as a GitHub-flavoured Markdown table."""
    out = ["| " + " | ".join(INVENTORY_FIELDS) + " |"]
    out.append("|" + "|".join(["---"] * len(INVENTORY_FIELDS)) + "|")
    for r in rows:
        cells = [
            r["unit_id"],
            "`" + r["config_sha256"][:16] + "`",
            str(r["schema_version"]) if r["schema_version"] is not None else "",
            _format_ts(r["first_seen"]),
            _format_ts(r["last_seen"]),
            str(r["boot_count"]),
            ";".join(sorted(r["components"])),
            ";".join(sorted(r["source_paths"])),
        ]
        out.append("| " + " | ".join(cells) + " |")
    return "\n".join(out) + "\n"


def cmd_inventory(args: argparse.Namespace) -> int:
    """Round 37 / BC-14: aggregate ``config_loaded`` events into a fleet
    inventory CSV / Markdown table.

    Walks each path in ``args.paths``: a directory is expanded to every
    ``audit*.jsonl`` file inside it (one level deep, so log-rotation
    siblings like ``audit.jsonl.1`` are picked up); a file is read
    directly. Aggregation is by ``(unit_id, config_sha256)`` so a unit
    that has been reflashed shows one row per distinct build it has
    booted.
    """
    candidate_files: list[Path] = []
    for raw in args.paths:
        p = Path(raw)
        if p.is_dir():
            # One-level glob; covers ``audit.jsonl`` + rotated siblings.
            candidate_files.extend(sorted(p.glob("audit*.jsonl*")))
        elif p.is_file():
            candidate_files.append(p)
        # Silently skip missing paths \u2014 a depot operator pointing at
        # a stale mount point should still get a usable report.
    rows = aggregate_inventory(iter_config_loaded_events(candidate_files))
    if args.format == "csv":
        sys.stdout.write(render_inventory_csv(rows))
    else:
        sys.stdout.write(render_inventory_markdown(rows))
    return EXIT_OK


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="lifetrac-config",
        description="Offline build-config tooling for LifeTrac v25.",
    )
    p.add_argument("--version", action="version", version=f"%(prog)s {TOOL_VERSION}")
    sub = p.add_subparsers(dest="command", required=True)

    pv = sub.add_parser("validate", help="schema-validate a TOML or bundle file")
    pv.add_argument("toml_path", help="path to build.<unit>.toml or installer bundle")
    pv.set_defaults(func=cmd_validate)

    pb = sub.add_parser("bundle", help="build an installer bundle from a TOML file")
    pb.add_argument("toml_path", help="path to validated build.<unit>.toml")
    pb.add_argument("-o", "--out-dir", required=True, help="directory to write bundle into")
    pb.add_argument(
        "--unit-id",
        help="override the unit_id label (must still match the TOML body's unit_id)",
    )
    pb.set_defaults(func=cmd_bundle)

    pver = sub.add_parser("verify", help="verify an installer bundle on disk")
    pver.add_argument("bundle_path", help="path to lifetrac-config-*.toml")
    pver.set_defaults(func=cmd_verify)

    pd = sub.add_parser("diff", help="show reload-class diff between two configs")
    pd.add_argument("old_path", help="baseline TOML or bundle")
    pd.add_argument("--against", required=True, help="candidate TOML or bundle")
    pd.set_defaults(func=cmd_diff)

    pp = sub.add_parser(
        "push",
        help="hand a bundle to the X8-side installer (local copy or SSH)",
    )
    pp.add_argument("bundle_path", help="path to lifetrac-config-*.toml")
    pp.add_argument(
        "--via",
        choices=("local", "ssh"),
        default="local",
        help="local copy into --dest (default) or scp+ssh trigger",
    )
    pp.add_argument(
        "--dest",
        help="destination directory (local: required; ssh: defaults to /var/lib/lifetrac/inbox)",
    )
    pp.add_argument("--host", help="ssh: hostname or IP of the X8")
    pp.add_argument("--user", help="ssh: remote user (default: lifetrac)")
    pp.add_argument(
        "--apply",
        action="store_true",
        help="local: invoke installer_daemon.process_mount() after copy",
    )
    pp.add_argument(
        "--target",
        help="local --apply: path to the active build.<unit>.toml to atomically replace",
    )
    pp.add_argument(
        "--execute",
        action="store_true",
        help="ssh: actually run the scp/ssh command (otherwise prints the plan)",
    )
    pp.set_defaults(func=cmd_push)

    # Round 31 / BC-03 -- firmware codegen.
    pc = sub.add_parser(
        "codegen",
        help="emit lifetrac_build_config.h for the firmware build (BC-03)",
    )
    pc.add_argument("toml_path", help="path to validated build.<unit>.toml or bundle")
    pc.add_argument("--out", required=True,
                    help="destination header path (e.g. firmware/common/lifetrac_build_config.h)")
    pc.add_argument("--check", action="store_true",
                    help="do not write; fail non-zero if --out drifts from the canonical output")
    pc.set_defaults(func=cmd_codegen)

    # Round 34 / BC-06 -- machine-readable config dump for non-Python consumers.
    pj = sub.add_parser(
        "dump-json",
        help="print the validated build config as canonical JSON (BC-06)",
    )
    pj.add_argument("toml_path", help="path to validated build.<unit>.toml or bundle")
    pj.set_defaults(func=cmd_dump_json)

    # Round 37 / BC-14 -- fleet-wide config inventory from audit.jsonl logs.
    pi = sub.add_parser(
        "inventory",
        help="aggregate config_loaded audit events into a fleet inventory (BC-14)",
    )
    pi.add_argument(
        "paths",
        nargs="+",
        help="audit.jsonl files and/or directories containing audit*.jsonl files",
    )
    pi.add_argument(
        "--format",
        choices=("csv", "markdown"),
        default="csv",
        help="output format (default: csv)",
    )
    pi.set_defaults(func=cmd_inventory)

    # Round 41 / BC-12C -- dry-run boot self-test against captured inventory.
    ps = sub.add_parser(
        "self-test",
        help="dry-run boot_self_test against a TOML config + inventory JSON",
    )
    ps.add_argument("toml_path", help="path to validated build.<unit>.toml")
    ps.add_argument("inventory_path",
                    help="path to JSON file matching HardwareInventory shape")
    ps.add_argument(
        "--format",
        choices=("text", "json"),
        default="text",
        help="output format (default: text)",
    )
    ps.set_defaults(func=cmd_self_test)
    return p


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (build_config.BuildConfigError, config_bundle.BundleError) as exc:
        print(f"ERROR  {exc}", file=sys.stderr)
        return EXIT_USAGE
    except FileNotFoundError as exc:
        print(f"ERROR  {exc}", file=sys.stderr)
        return EXIT_USAGE


if __name__ == "__main__":
    sys.exit(main())
