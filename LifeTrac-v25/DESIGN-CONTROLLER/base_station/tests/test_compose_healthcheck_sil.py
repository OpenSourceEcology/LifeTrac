"""SIL gate for the L1 compose healthchecks -- TODO.md PWR-3.

The healthchecks REPORT STATUS ONLY: Docker never restarts a container for
being "unhealthy". lifetrac-supervisor (firmware/x8_recovery/, L3, PWR-4)
reads the status and restarts allowlisted containers. This file pins the
compose side, from the files in the repo; no docker daemon is involved.

Static checks (PyYAML ``safe_load``):

* ``docker-compose.yml`` (base) and ``firmware/tractor_x8/docker-compose.yml``
  (tractor production stack) parse.
* mosquitto (both files) and web_ui (base) have a healthcheck with test,
  interval, timeout, retries and start_period. web_ui's start_period covers
  the IP-201 broker-connect budget that web_ui.py spends before uvicorn binds.
* No service in any compose file uses ``start_interval``: it needs Docker
  25.0, and the boards run 24.0.2 (base) and 20.10.14 (tractor).
* No healthcheck on lora_bridge (KISS, obsolete: RS-5.11), audit_tail (idle
  by design) or the tractor camera (held until RS-5.13).
* ``$SYS`` is written ``$$SYS``, or compose would substitute a host variable
  named SYS (empty) into the topic.
* The mosquitto probe is exec form (no shell, so nothing from the image but
  ``mosquitto_sub``) and connects to host ``mosquitto``: the service name the
  clients use, which Docker's embedded DNS resolves to the broker's
  compose-network address, never loopback. The broker keeps a compose
  network (no ``network_mode``) and no ``extra_hosts`` entry overrides it.
* The tractor broker binds the repo ``mosquitto.conf`` read-only and no
  volume sits on /mosquitto/config any more. Every broker config a compose
  file mounts has a non-loopback listener (the probe uses the container's
  network address) and keeps the $SYS tree on; the tractor one also has
  allow_anonymous.
* No Dockerfile under DESIGN-CONTROLLER has a HEALTHCHECK: the bench harness
  runs the same images with bare ``docker run``.
* web_ui.py registers ``GET /api/session`` once, on ``app``, with no
  dependency at handler, route or app level, no middleware outside a short
  allowlist, and no mount or rebinding of ``app`` in front of it.

Behaviour checks, run on this machine (no root, no /dev, /proc or /etc
access, no docker daemon, nothing leaves loopback):

* the real ``docker compose config`` (the v2 plugin binary, called directly;
  skipped where it is not installed) keeps both probes as written, with a
  host ``SYS=WRONG`` in its environment that must not leak in;
* the mosquitto exec-form argv, compose-unescaped, run without a shell
  against a fake ``mosquitto_sub``;
* the real web_ui app (paho stubbed, as the other web_ui tests do) answers an
  anonymous ``GET /api/session`` with 200 (skipped without fastapi/httpx);
* the web_ui ``python -c`` probe against a stub HTTP server on an ephemeral
  loopback port, with a dead http_proxy set to prove the probe ignores it.

Skipped without PyYAML (a ``skipif`` mark, not ``pytest.importorskip``, so
``python -m unittest`` collects nothing and run_tests.ps1 falls back to
pytest instead of reporting a crash); the fake-``mosquitto_sub`` cases also
skip where there is no POSIX sh (the Windows bench laptop).
"""

from __future__ import annotations

import ast
import importlib
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

import pytest

try:
    import yaml
except ImportError:  # pragma: no cover
    # CI has it: requirements-dev.txt -> requirements.txt -> uvicorn[standard],
    # whose extras include PyYAML (requirements.lock.txt pins 6.0.3).
    yaml = None

# A module-level pytest.importorskip raises pytest's Skipped, a BaseException,
# which `python -m unittest` (run_tests.ps1's first pass) reports as a crash.
pytestmark = pytest.mark.skipif(yaml is None, reason="PyYAML not installed")

_BS = Path(__file__).resolve().parents[1]
_DC = _BS.parent
_BASE_COMPOSE = _DC / "docker-compose.yml"
_TRACTOR_DIR = _DC / "firmware" / "tractor_x8"
_TRACTOR_COMPOSE = _TRACTOR_DIR / "docker-compose.yml"
_TRACTOR_CONF = _TRACTOR_DIR / "mosquitto.conf"
_WEB_UI = _BS / "web_ui.py"

_COMPOSE_FILES = {"base": _BASE_COMPOSE, "tractor": _TRACTOR_COMPOSE}
_CONF_TARGET = "/mosquitto/config/mosquitto.conf"
_HEALTH_KEYS = ("test", "interval", "timeout", "retries", "start_period")

# (file, service) pairs that must report health, and the ones that must not.
_MUST_HAVE = [("base", "mosquitto"), ("base", "web_ui"),
              ("tractor", "mosquitto")]
# Services that reach the broker as LIFETRAC_MQTT_HOST; the probe uses the
# same name.
_BROKER_CLIENTS = {"base": ("lora_bridge", "web_ui", "audit_tail"),
                   "tractor": ("camera",)}
_MUST_NOT = [
    ("base", "lora_bridge", "KISS bridge is obsolete (RS-5.11)"),
    ("base", "audit_tail", "idle by design, nothing to probe"),
    ("tractor", "camera", "held until RS-5.13"),
]

_HAVE_SH = os.name == "posix" and shutil.which("sh") is not None


def _load(name: str) -> dict:
    with _COMPOSE_FILES[name].open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _service(name: str, svc: str) -> dict:
    services = _load(name)["services"]
    assert svc in services, f"{name} compose has no service {svc!r}"
    return services[svc]


def _strings(node):
    """Every string in a parsed YAML tree, keys included."""
    if isinstance(node, str):
        yield node
    elif isinstance(node, dict):
        for k, v in node.items():
            yield from _strings(k)
            yield from _strings(v)
    elif isinstance(node, list):
        for v in node:
            yield from _strings(v)


_DUR = re.compile(r"(\d+(?:\.\d+)?)(us|ms|s|m|h)")
_DUR_S = {"us": 1e-6, "ms": 1e-3, "s": 1.0, "m": 60.0, "h": 3600.0}


def _seconds(value) -> float:
    """A compose duration ("30s", "1m30s") in seconds."""
    text = str(value).strip()
    parts = _DUR.findall(text)
    assert parts and "".join(n + u for n, u in parts) == text, \
        f"not a compose duration: {value!r}"
    return sum(float(n) * _DUR_S[u] for n, u in parts)


def _compose_unescape(text: str) -> str:
    """What the engine gets after compose interpolation, for a probe string.

    `$$` is compose's escape for one literal `$`; any other `$` starts a
    host-variable substitution, which a probe must never depend on, so every
    `$` in a probe must come doubled.
    """
    out = []
    i = 0
    while i < len(text):
        if text[i] == "$":
            assert text[i + 1:i + 2] == "$", (
                f"single '$' at offset {i} in {text!r}: compose would "
                "interpolate it from the host environment")
            out.append("$")
            i += 2
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def _healthcheck(name: str, svc: str) -> dict:
    hc = _service(name, svc).get("healthcheck")
    assert isinstance(hc, dict), f"{name}/{svc} has no healthcheck"
    return hc


def _mosquitto_argv(name: str) -> list:
    """The broker probe's argv as the engine runs it (compose-unescaped)."""
    test = _healthcheck(name, "mosquitto")["test"]
    assert isinstance(test, list) and test[0] == "CMD" and len(test) > 1, (
        f"{name}/mosquitto probe should be exec form [CMD, ...], got {test!r}: "
        "CMD-SHELL would lean on the image's shell and busybox applets")
    return [_compose_unescape(str(a)) for a in test[1:]]


def _arg(args, flag):
    assert flag in args, f"{flag} missing from {args!r}"
    return args[args.index(flag) + 1]


def _env(svc: dict) -> dict:
    env = svc.get("environment") or {}
    if isinstance(env, list):
        env = dict(str(e).split("=", 1) for e in env)
    return {k: str(v) for k, v in env.items()}


def _web_ui_probe() -> str:
    test = _healthcheck("base", "web_ui")["test"]
    assert isinstance(test, list) and test[:3] == ["CMD", "python", "-c"] \
        and len(test) == 4, f"web_ui probe should be CMD python -c, got {test!r}"
    return test[3]


def _volume_entries(svc: dict):
    """(source, target, read_only) for each volume entry of a service."""
    for v in svc.get("volumes") or []:
        if isinstance(v, dict):
            yield (v.get("source"), v.get("target"),
                   bool(v.get("read_only")))
        else:
            parts = str(v).split(":")
            src, dst = (parts[0], parts[1]) if len(parts) > 1 else (None, parts[0])
            mode = parts[2] if len(parts) > 2 else ""
            yield src, dst, "ro" in mode.split(",")


def _broker_conf_path(name: str) -> Path:
    """The host file a compose file binds over the broker's config."""
    hits = [(src, ro) for src, dst, ro in _volume_entries(_service(name, "mosquitto"))
            if dst == _CONF_TARGET]
    assert len(hits) == 1, f"{name}: expected one bind on {_CONF_TARGET}, got {hits!r}"
    src, ro = hits[0]
    assert src and src.startswith("."), \
        f"{name}: broker config should be a repo-relative bind, got {src!r}"
    assert ro, f"{name}: {src} must be mounted read-only (:ro)"
    return (_COMPOSE_FILES[name].parent / src).resolve()


def _conf_directives(path: Path) -> list[list[str]]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            rows.append(line.split())
    return rows


_LOOPBACK = ("127.", "localhost", "::1")


# ---- static: both files, required keys ------------------------------------

@pytest.mark.parametrize("name", sorted(_COMPOSE_FILES))
def test_compose_file_parses(name):
    data = _load(name)
    assert isinstance(data, dict) and isinstance(data.get("services"), dict)
    assert "mosquitto" in data["services"]


@pytest.mark.parametrize("name,svc", _MUST_HAVE)
def test_healthcheck_has_all_keys(name, svc):
    hc = _healthcheck(name, svc)
    missing = [k for k in _HEALTH_KEYS if k not in hc]
    assert not missing, f"{name}/{svc} healthcheck lacks {missing}"
    assert not hc.get("disable"), f"{name}/{svc} healthcheck is disabled"
    assert hc["test"][0] in ("CMD", "CMD-SHELL")
    interval, timeout = _seconds(hc["interval"]), _seconds(hc["timeout"])
    assert 0 < timeout < interval, \
        f"{name}/{svc}: timeout {timeout}s must be below interval {interval}s"
    assert isinstance(hc["retries"], int) and hc["retries"] >= 1
    assert _seconds(hc["start_period"]) >= 20


def test_web_ui_start_period_covers_broker_connect_budget():
    """web_ui.py blocks at import in _connect_mqtt_with_retry (IP-201) before
    uvicorn binds :8080; the start_period must outlast that budget."""
    src = _WEB_UI.read_text(encoding="utf-8")
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == "_connect_mqtt_with_retry")
    m = re.search(r"deadline\s*=\s*time\.monotonic\(\)\s*\+\s*([0-9.]+)",
                  ast.get_source_segment(src, fn) or "")
    assert m, "IP-201 connect deadline not found in web_ui.py"
    budget = float(m.group(1))
    start = _seconds(_healthcheck("base", "web_ui")["start_period"])
    assert start > budget, \
        f"web_ui start_period {start}s does not cover the {budget}s connect budget"


def test_no_start_interval_in_any_compose_file():
    # bench-evidence/ holds snapshots of what was on the boards; not ours to fix.
    # "compose" anywhere in the name, not only as a prefix: overrides such as
    # rs13_2_desk.compose.yml merge onto the base file on the 24.0.2 engine
    # (RS13_2_DESK_CHECK.md), so their healthchecks reach a board too.
    files = [p for p in _DC.rglob("*.y*ml")
             if "bench-evidence" not in p.parts
             and p.suffix in (".yml", ".yaml") and "compose" in p.name]
    assert _BASE_COMPOSE in files and _TRACTOR_COMPOSE in files
    assert _DC / "firmware" / "x8_lora_bootloader_helper" / "bench_tools" \
        / "rs13_2_desk.compose.yml" in files
    for path in files:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        for svc, body in (data.get("services") or {}).items():
            hc = (body or {}).get("healthcheck") or {}
            assert "start_interval" not in hc, (
                f"{path.relative_to(_DC)}:{svc} uses start_interval "
                "(Docker 25.0+; the boards run 20.10 / 24.0)")


@pytest.mark.parametrize("name,svc,why", _MUST_NOT)
def test_no_healthcheck_where_there_is_nothing_to_probe(name, svc, why):
    assert "healthcheck" not in _service(name, svc), f"{name}/{svc}: {why}"


# ---- static: $SYS escaping ------------------------------------------------

@pytest.mark.parametrize("name", sorted(_COMPOSE_FILES))
def test_sys_topic_is_compose_escaped(name):
    for s in _strings(_load(name)):
        for run in re.findall(r"(\$+)SYS", s):
            assert len(run) % 2 == 0, \
                f"{name}: {s!r} has an unescaped $SYS (write $$SYS)"


# ---- static: the mosquitto probe --------------------------------------------

@pytest.mark.parametrize("name", sorted(_COMPOSE_FILES))
def test_mosquitto_probe_reads_sys_from_the_service_name(name):
    argv = _mosquitto_argv(name)
    assert argv[0] == "mosquitto_sub"
    # The service key is the DNS alias compose gives the broker container.
    host = _arg(argv, "-h")
    assert host == "mosquitto" and host in _load(name)["services"]
    assert not host.startswith(_LOOPBACK)
    assert _arg(argv, "-p") == "1883"
    assert _arg(argv, "-t") == "$SYS/broker/version"
    # mosquitto publishes $SYS/broker/version once, retained, at broker start
    # (src/sys_tree.c, sys_tree__init: retain=1, no expiry; same in 2.0.18,
    # 2.0.20, 2.0.22), so the probe only ever receives it as a retained
    # message. -R (no_retain) and a -T filter-out matching it both return in
    # mosquitto_sub's message callback before -C counts the message
    # (client/sub_client.c), so every probe would sit out -W, exit 27, and
    # the broker would read unhealthy for good. The fake mosquitto_sub below
    # cannot show this; this assert is the only guard.
    assert argv.count("-t") == 1
    for flag in ("-R", "-T", "--filter-out"):
        assert flag not in argv, f"{name}: {flag} would drop the retained topic"
    assert _arg(argv, "-C") == "1"
    # A fixed client id names the probe in the broker log.
    assert _arg(argv, "-i") == "lifetrac-healthcheck"
    # mosquitto_sub's own timeout must fire before docker's kills it.
    timeout = _seconds(_healthcheck(name, "mosquitto")["timeout"])
    assert 0 < int(_arg(argv, "-W")) < timeout


@pytest.mark.parametrize("name", sorted(_COMPOSE_FILES))
def test_probe_host_is_the_name_the_clients_use(name):
    host = _arg(_mosquitto_argv(name), "-h")
    for svc in _BROKER_CLIENTS[name]:
        assert _env(_service(name, svc)).get("LIFETRAC_MQTT_HOST") == host, \
            f"{name}/{svc} reaches the broker by another name than the probe"


@pytest.mark.parametrize("name", sorted(_COMPOSE_FILES))
def test_broker_alias_comes_from_a_compose_network(name):
    # network_mode host/bridge/none/container: has no embedded-DNS alias, so
    # `mosquitto` would not resolve (or would resolve elsewhere).
    svc = _service(name, "mosquitto")
    assert "network_mode" not in svc
    extra = svc.get("extra_hosts") or []
    names = (list(extra) if isinstance(extra, dict)
             else [str(e).split(":", 1)[0].split("=", 1)[0] for e in extra])
    assert "mosquitto" not in [n.strip() for n in names]


# ---- static: broker configs and mounts --------------------------------------

def test_tractor_binds_repo_conf_read_only_and_no_config_volume():
    svc = _service("tractor", "mosquitto")
    assert _broker_conf_path("tractor") == _TRACTOR_CONF.resolve()
    assert _TRACTOR_CONF.is_file()
    # A volume on /mosquitto/config would hold the image's default config
    # under the bind; the layout must not depend on Docker's mount order.
    targets = [dst for _src, dst, _ro in _volume_entries(svc)]
    assert "/mosquitto/config" not in [t.rstrip("/") for t in targets]
    assert "mosquitto_config" not in (_load("tractor").get("volumes") or {})


@pytest.mark.parametrize("name", sorted(_COMPOSE_FILES))
def test_broker_conf_has_non_loopback_listener_and_sys_tree(name):
    path = _broker_conf_path(name)
    rows = _conf_directives(path)
    listeners = [r for r in rows if r[0] == "listener"]
    assert listeners, f"{path}: no listener line (mosquitto 2 = local-only mode)"
    open_ = [r for r in listeners
             if r[1] == "1883" and (len(r) < 3 or not r[2].startswith(_LOOPBACK))]
    assert open_, f"{path}: no non-loopback listener on 1883: {listeners!r}"
    assert ["sys_interval", "0"] not in rows, \
        f"{path}: sys_interval 0 turns off $SYS and breaks the healthcheck"


def test_tractor_conf_allows_the_camera_in():
    rows = _conf_directives(_TRACTOR_CONF)
    assert ["allow_anonymous", "true"] in rows
    env = _service("tractor", "camera").get("environment") or []
    if isinstance(env, list):
        env = dict(e.split("=", 1) for e in env)
    assert env.get("LIFETRAC_MQTT_HOST") == "mosquitto"
    assert env.get("LIFETRAC_MQTT_PORT", "1883") == "1883"


def test_tractor_conf_lf_endings():
    assert b"\r" not in _TRACTOR_CONF.read_bytes()


def test_no_dockerfile_healthcheck():
    files = [p for p in _DC.rglob("*") if p.is_file()
             and (p.name.startswith(("Dockerfile", "Containerfile"))
                  or p.name.endswith(".Dockerfile"))]
    assert (_DC / "Dockerfile") in files
    for path in files:
        text = path.read_text(encoding="utf-8", errors="replace")
        assert not re.search(r"(?im)^\s*HEALTHCHECK\b", text), (
            f"{path.relative_to(_DC)} has a HEALTHCHECK; healthchecks live in "
            "the compose files (the bench harness runs the images bare)")


# Route registrars that could claim /api/session (FastAPI and Starlette).
_ROUTE_CALLS = {"get", "post", "put", "patch", "delete", "head", "options",
                "trace", "api_route", "route", "add_api_route", "add_route",
                "websocket", "websocket_route", "add_websocket_route"}
# Middleware that cannot turn an anonymous GET into a 4xx/5xx. Anything else
# (TrustedHostMiddleware, an auth or CSRF layer, HTTPSRedirect) can fail the
# probe: check it passes GET /api/session from 127.0.0.1:8080, then list it.
_MIDDLEWARE_OK = {"GZipMiddleware", "CORSMiddleware"}


def _callee(call: ast.Call) -> str:
    f = call.func
    return f.id if isinstance(f, ast.Name) else getattr(f, "attr", "")


def _kwarg(call: ast.Call, name: str):
    return next((k.value for k in call.keywords if k.arg == name), None)


def test_web_ui_session_route_is_get_and_unauthenticated():
    tree = ast.parse(_WEB_UI.read_text(encoding="utf-8"))
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]

    regs = [c for c in calls if isinstance(c.func, ast.Attribute)
            and c.func.attr in _ROUTE_CALLS and c.args
            and isinstance(c.args[0], ast.Constant)
            and c.args[0].value == "/api/session"]
    assert len(regs) == 1, \
        f"/api/session registered {len(regs)} times; the first one wins"
    reg = regs[0]
    assert reg.func.attr == "get" and isinstance(reg.func.value, ast.Name) \
        and reg.func.value.id == "app", "/api/session is not @app.get"
    assert _kwarg(reg, "dependencies") is None, \
        "@app.get('/api/session', dependencies=...): the probe would 401"

    handler = next(n for n in ast.walk(tree)
                   if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                   and reg in n.decorator_list)
    defaults = handler.args.defaults + [d for d in handler.args.kw_defaults if d]
    deps = [d for d in defaults if isinstance(d, ast.Call)
            and _callee(d) in ("Depends", "Security")]
    assert not deps, "/api/session gained an auth dependency; the probe would 401"

    # App level: dependencies or middleware on the constructor, a later
    # `.dependencies` edit, add_middleware / @app.middleware, a mount in front
    # of /api, or `app` rebound to a wrapper (app = SomeMiddleware(app)).
    ctors = [c for c in calls if _callee(c) == "FastAPI"]
    assert len(ctors) == 1
    for kw in ("dependencies", "middleware"):
        assert _kwarg(ctors[0], kw) is None, f"FastAPI({kw}=...) covers /api/session"
    assert not [n for n in ast.walk(tree)
                if isinstance(n, ast.Attribute) and n.attr == "dependencies"], \
        "web_ui.py edits a .dependencies list; check it spares /api/session"
    middleware = []
    for c in calls:
        if not isinstance(c.func, ast.Attribute):
            continue
        if c.func.attr == "add_middleware":
            arg = c.args[0] if c.args else None
            middleware.append(getattr(arg, "id", None) or getattr(arg, "attr", "?"))
        elif c.func.attr == "middleware":
            middleware.append("@app.middleware")
        elif c.func.attr == "mount" and c.args \
                and isinstance(c.args[0], ast.Constant):
            prefix = str(c.args[0].value).rstrip("/")
            assert not ("/api/session" + "/").startswith(prefix + "/"), \
                f"app.mount({c.args[0].value!r}) sits in front of /api/session"
    bad = [m for m in middleware if m not in _MIDDLEWARE_OK]
    assert not bad, f"web_ui gained middleware {bad}; see _MIDDLEWARE_OK"
    app_binds = [n for n in tree.body if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == "app"
                         for t in n.targets)]
    assert len(app_binds) == 1 and isinstance(app_binds[0].value, ast.Call) \
        and _callee(app_binds[0].value) == "FastAPI", "`app` is rebound"

    cmd = _service("base", "web_ui")["command"]
    assert cmd[cmd.index("--port") + 1] == "8080"
    assert "'http://127.0.0.1:8080/api/session'" in _web_ui_probe()


def test_web_ui_app_serves_session_to_an_anonymous_probe(monkeypatch, tmp_path):
    """The real app, not its source text: catches any auth layer the static
    test above cannot see. paho is stubbed as in test_web_ui_auth.py, so
    the import-time broker connect returns at once. The import writes a
    config_loaded audit record; it goes to tmp_path, not the machine's
    /var/log/lifetrac/audit.jsonl (2026-10-17 review)."""
    pytest.importorskip("paho.mqtt.client")
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient

    monkeypatch.syspath_prepend(str(_BS))
    # web_ui._get_audit_log() reads audit_log.DEFAULT_PATH when it first
    # opens the log, and the reload below resets its cached _AUDIT_LOG.
    monkeypatch.setattr("audit_log.DEFAULT_PATH", str(tmp_path / "audit.jsonl"))
    if not os.environ.get("LIFETRAC_PIN"):
        monkeypatch.setenv("LIFETRAC_PIN", "424242")
    with mock.patch("paho.mqtt.client.Client") as mqtt_class:
        inst = mqtt_class.return_value
        inst.connect = mock.MagicMock(return_value=None)
        inst.loop_start = mock.MagicMock()
        inst.subscribe = mock.MagicMock()
        inst.publish = mock.MagicMock()
        if "web_ui" in sys.modules:
            web_ui = importlib.reload(sys.modules["web_ui"])
        else:
            web_ui = importlib.import_module("web_ui")
    # Same Host the probe sends; no cookie, no credentials. Not used as a
    # context manager, so the app's startup hooks do not run (as in the
    # other web_ui tests).
    client = TestClient(web_ui.app, base_url="http://127.0.0.1:8080")
    r = client.get("/api/session")
    assert r.status_code == 200, (r.status_code, r.text)
    assert r.json() == {"authenticated": False}


# ---- behaviour: the real compose, and the mosquitto argv --------------------

def _compose_plugin():
    """The docker compose v2 plugin binary, if installed. It runs standalone
    and `config` reads files only (no docker CLI, no daemon). Override with
    LIFETRAC_COMPOSE_BIN."""
    override = os.environ.get("LIFETRAC_COMPOSE_BIN")
    candidates = [override] if override else [
        "/usr/libexec/docker/cli-plugins/docker-compose",
        "/usr/lib/docker/cli-plugins/docker-compose",
        "/usr/local/lib/docker/cli-plugins/docker-compose",
        str(Path.home() / ".docker" / "cli-plugins" / "docker-compose"),
    ]
    for c in candidates:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


@pytest.mark.parametrize("name", sorted(_COMPOSE_FILES))
def test_real_compose_keeps_the_probes(name, tmp_path):
    exe = _compose_plugin()
    if exe is None:
        pytest.skip("docker compose v2 plugin not installed")
    path = _COMPOSE_FILES[name]
    env = {
        "PATH": os.environ.get("PATH", ""),
        "HOME": str(tmp_path),
        "DOCKER_CONFIG": str(tmp_path / "docker"),
        # config must not need a daemon; if it tried, it would find none.
        "DOCKER_HOST": f"unix://{tmp_path / 'no-daemon.sock'}",
        # An unescaped $SYS would pick this up.
        "SYS": "WRONG",
    }
    proc = subprocess.run(
        [exe, "-f", str(path), "--project-directory", str(path.parent),
         "config", "--format", "json"],
        env=env, capture_output=True, text=True, timeout=120)
    if proc.returncode != 0 and "secret" in proc.stderr.lower():
        pytest.skip(f"compose wants the ./secrets files: {proc.stderr.strip()}")
    assert proc.returncode == 0, proc.stderr
    services = json.loads(proc.stdout)["services"]
    for file_name, svc in _MUST_HAVE:
        if file_name != name:
            continue
        got, want = services[svc]["healthcheck"], _healthcheck(name, svc)
        # compose prints each `$` re-escaped as `$$`, so equality with the
        # file also shows SYS=WRONG never got in.
        assert got["test"] == want["test"], f"{name}/{svc}: {got['test']!r}"
        assert "WRONG" not in json.dumps(got)
        for k in ("interval", "timeout", "start_period"):
            assert _seconds(got[k]) == _seconds(want[k]), (name, svc, k)
        assert got["retries"] == want["retries"]
        assert "start_interval" not in got


def _write_exe(path: Path, body: str) -> None:
    path.write_text(body, encoding="utf-8", newline="\n")
    path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


@pytest.mark.skipif(not _HAVE_SH, reason="needs a POSIX sh for the fake")
@pytest.mark.parametrize("name", sorted(_COMPOSE_FILES))
@pytest.mark.parametrize("sub_rc", [0, 1, 27])
def test_probe_argv_reaches_mosquitto_sub_unchanged(name, tmp_path, sub_rc):
    # Docker execs CMD-form argv directly (no shell), so the topic arrives
    # literally and mosquitto_sub's own exit code is the verdict: 0 healthy,
    # anything else a failed probe (27 is its -W timeout).
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "sub.log"
    _write_exe(bin_dir / "mosquitto_sub",
               '#!/bin/sh\nfor a in "$@"; do printf "%s\\n" "$a"; done > "$FAKE_SUB_LOG"\n'
               'exit "${FAKE_SUB_RC:-0}"\n')
    argv = _mosquitto_argv(name)
    exe = shutil.which(argv[0], path=str(bin_dir))
    assert exe, "fake mosquitto_sub not found"
    env = {"PATH": f"{bin_dir}{os.pathsep}{os.environ.get('PATH', '')}",
           "FAKE_SUB_LOG": str(log), "FAKE_SUB_RC": str(sub_rc),
           "SYS": "WRONG"}
    proc = subprocess.run([exe] + argv[1:], env=env, capture_output=True,
                          text=True, timeout=30)
    assert log.read_text(encoding="utf-8").splitlines() == argv[1:]
    assert _arg(argv, "-t") == "$SYS/broker/version"
    assert proc.returncode == sub_rc


# ---- behaviour: the web_ui probe against a loopback stub --------------------

class _Stub(BaseHTTPRequestHandler):
    status_code = 200
    seen: list = []

    def do_GET(self):  # noqa: N802 (http.server API)
        type(self).seen.append(self.path)
        code = type(self).status_code if self.path == "/api/session" else 404
        body = b'{"authenticated": false}'
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        pass


def _run_web_probe(port: int) -> int:
    code = _web_ui_probe().replace("127.0.0.1:8080/", f"127.0.0.1:{port}/")
    env = {k: v for k, v in os.environ.items()
           if k.lower() not in ("no_proxy", "pythonpath")}
    # A dead proxy: if the probe honoured http_proxy it would fail.
    env.update(http_proxy="http://127.0.0.1:9", HTTP_PROXY="http://127.0.0.1:9")
    proc = subprocess.run([sys.executable, "-c", code], env=env,
                          capture_output=True, text=True, timeout=60)
    return proc.returncode


@pytest.fixture
def stub_server():
    _Stub.seen = []
    _Stub.status_code = 200
    srv = ThreadingHTTPServer(("127.0.0.1", 0), _Stub)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    yield srv
    srv.shutdown()
    srv.server_close()


def test_web_probe_passes_on_200_and_ignores_http_proxy(stub_server):
    assert _run_web_probe(stub_server.server_address[1]) == 0
    assert _Stub.seen == ["/api/session"]


@pytest.mark.parametrize("status_code", [401, 500, 503])
def test_web_probe_fails_on_error_status(stub_server, status_code):
    _Stub.status_code = status_code
    assert _run_web_probe(stub_server.server_address[1]) != 0


def test_web_probe_fails_when_nothing_listens():
    srv = ThreadingHTTPServer(("127.0.0.1", 0), _Stub)
    port = srv.server_address[1]
    srv.server_close()
    assert _run_web_probe(port) != 0
