"""SIL gate for the L3 host supervisor -- TODO.md PWR-4.

Drives ``firmware/x8_recovery/lifetrac-supervisor.sh`` (POSIX sh, runs on
the X8 host) through its ``once`` command, one invocation per health pass,
and, for what only a long-lived process can show (disarm and clear-hold
taking effect on the next pass), through its ``run`` loop with
CHECK_INTERVAL_S=1 in a bounded child process group that the test stops.
Everything the script would touch on a board is redirected into a temp dir:

* ``docker``, ``systemctl``, ``fw_printenv``, ``systemd-notify`` and ``sync``
  are fake scripts on a PATH that holds nothing else but a handful of real
  coreutils (``timeout``, ``stat``, ``mv``, ``rm``, ``mkdir``, ``sleep``,
  ``cat``). Each fake appends its argv to one log, so the tests can check
  what was called and in which order. The fake ``systemctl`` also records
  which ``last_action`` and ``consecutive_reboots`` were on disk at the
  moment it was called, and the fake ``sync`` records which ``*.tmp`` files
  existed when it ran (so "tmp, sync, rename" is checked, not just "a sync
  happened somewhere earlier"). It also keeps a copy of ``last_action`` as
  it was at each reboot call (what the next boot reads if that reboot
  really happened: a real one never returns) and can move the uptime on
  when ``systemctl reboot`` is accepted (a slow rung). The fake ``docker
  inspect`` prints ``/<name>`` first, like the real ``{{.Name}}``, unless a
  test makes the name resolve to another container.
* uptime and boot id come from files the test writes
  (``LIFETRAC_PROC_UPTIME``, ``LIFETRAC_BOOT_ID_FILE``), so "time passing"
  and "the board rebooted" are just file writes. Real sleeps only in the
  ``run`` loop tests and one SYSRQ_SETTLE_S=1 test (a few seconds each).
* the sysrq trigger is a regular file (``LIFETRAC_SYSRQ``); the script
  appends one character per write, so the file content is the write order.
* the tractor's H747 link service (D-PWR1) is ``_FakeLink``, a thread that
  rewrites the idle file every few ms with a fresh <seq> and answers a
  reboot request according to its mode; the supervisor polls it every
  20 ms (``LIFETRAC_IDLE_POLL``) instead of every second.

No root, no /dev, /proc or /etc access, no network. Skipped where there is
no POSIX sh (the Windows bench laptop). The whole suite runs under /bin/sh
(dash on Ubuntu CI) and again under bash when bash is installed, because
LmP's /bin/sh may be either.

Test classes:

* X8SUP_A_Static: syntax under sh/dash/bash, shellcheck when installed, LF
  endings, executable bit, the never-do list holds in code, every external
  command runs under ``timeout``, unit file and example config contents.
* X8SUP_B_Supervisor: the behaviour, under /bin/sh.
* X8SUP_C_SupervisorBash: the same behaviour tests under bash.
"""

from __future__ import annotations

import os
import re
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_DC = _BS.parent
_X8R = _DC / "firmware" / "x8_recovery"
_SCRIPT = _X8R / "lifetrac-supervisor.sh"
_UNIT = _X8R / "lifetrac-supervisor.service"
_CONF_EXAMPLE = _X8R / "supervisor.conf.example"

_HAVE_POSIX = (
    os.name == "posix"
    and shutil.which("sh") is not None
    and shutil.which("timeout") is not None
)
_REAL_TOOLS = ("timeout", "stat", "mv", "rm", "mkdir", "sleep", "cat")

BOOT_A = "aaaaaaaa-1111-4111-8111-000000000001"
BOOT_B = "bbbbbbbb-2222-4222-8222-000000000002"
BOOT_C = "cccccccc-3333-4333-8333-000000000003"

_FAKE_DOCKER = r"""#!/bin/sh
printf 'docker %s\n' "$*" >> "$FAKE_LOG"
last=
for a in "$@"; do last=$a; done
case $1 in
  info)
    rc=0
    [ -f "$FAKE_DIR/docker_info_rc" ] && read -r rc < "$FAKE_DIR/docker_info_rc"
    exit "$rc" ;;
  inspect)
    if [ -f "$FAKE_DIR/containers/$last" ]; then
      name=/$last
      if [ -f "$FAKE_DIR/containers/$last.resolves" ]; then
        read -r name < "$FAKE_DIR/containers/$last.resolves"
      fi
      printf '%s ' "$name"
      cat "$FAKE_DIR/containers/$last"
      exit 0
    fi
    echo "Error: No such container: $last" >&2
    exit 1 ;;
  restart)
    exit 0 ;;
esac
exit 0
"""

_FAKE_SYSTEMCTL = r"""#!/bin/sh
act=none
if [ -f "$FAKE_STATE/last_action" ]; then
  while IFS= read -r l; do
    case $l in action=*) act=${l#action=} ;; esac
  done < "$FAKE_STATE/last_action"
fi
n=none
if [ -f "$FAKE_STATE/consecutive_reboots" ]; then
  read -r n < "$FAKE_STATE/consecutive_reboots"
fi
printf 'systemctl %s [last_action=%s consecutive=%s]\n' "$*" "$act" "$n" >> "$FAKE_LOG"
# The record on disk at the moment of a reboot call: what the next boot reads
# if this reboot really happened (a real one never returns).
if [ -f "$FAKE_STATE/last_action" ]; then
  case $* in
    reboot) cat "$FAKE_STATE/last_action" > "$FAKE_DIR/last_action.at_reboot" ;;
    "reboot -ff") cat "$FAKE_STATE/last_action" > "$FAKE_DIR/last_action.at_reboot_ff" ;;
  esac
fi
# The tractor's motion-hold request on disk at that moment (D-PWR1).
if [ -f "$FAKE_REQ" ]; then
  case $* in
    reboot) cat "$FAKE_REQ" > "$FAKE_DIR/request.at_reboot" ;;
    "reboot -ff") cat "$FAKE_REQ" > "$FAKE_DIR/request.at_reboot_ff" ;;
  esac
fi
# A slow rung: uptime has moved on by the time systemd takes the reboot.
if [ "$*" = reboot ] && [ -f "$FAKE_DIR/uptime_at_accept" ]; then
  cat "$FAKE_DIR/uptime_at_accept" > "$FAKE_UPTIME"
fi
rc=0
if [ "$*" = reboot ] && [ -f "$FAKE_DIR/systemctl_reboot_rc" ]; then
  read -r rc < "$FAKE_DIR/systemctl_reboot_rc"
fi
exit "$rc"
"""

_FAKE_FW_PRINTENV = r"""#!/bin/sh
printf 'fw_printenv %s\n' "$*" >> "$FAKE_LOG"
[ "$1" = -n ] || exit 1
if [ -f "$FAKE_DIR/fw_$2" ]; then
  cat "$FAKE_DIR/fw_$2"
  exit 0
fi
echo "## Error: \"$2\" not defined" >&2
exit 1
"""

_FAKE_NOTIFY = r"""#!/bin/sh
printf 'systemd-notify %s\n' "$*" >> "$FAKE_LOG"
exit 0
"""

_FAKE_SYNC = r"""#!/bin/sh
tmps=
for f in "$FAKE_STATE"/*.tmp "$FAKE_STATE"/per_boot/*.tmp; do
  if [ -e "$f" ]; then
    tmps="$tmps ${f##*/}"
  fi
done
printf 'sync [tmp=%s]\n' "${tmps# }" >> "$FAKE_LOG"
exit 0
"""

_DEFAULT_CONF = {
    "ROLE": "base",
    "CHECK_INTERVAL_S": "30",
    "FAIL_THRESHOLD": "3",
    "CONTAINERS": "",
    "HEARTBEATS": "",
    "CHECK_DOCKER": "1",
    "CONTAINER_RESTART_MAX": "3",
    "REBOOT_MAX_CONSECUTIVE": "3",
    "HEALTHY_RESET_S": "1800",
    "REBOOT_GRACE_S": "600",
    "FORCE_REBOOT_AFTER_S": "180",
    "SYSRQ_SETTLE_S": "0",
    "TRACTOR_REBOOT_ENABLED": "0",
}


class _FakeLink:
    """The tractor's H747 link service (D-PWR1), faked. Every few ms it
    rewrites the idle file (tmp + rename) with a new <seq>; ``mode`` says what
    the H747 reports: ``grant`` (idle, and ``locked <seq> <id>`` while a
    request file exists), ``idle`` (never takes a lock), ``busy`` or
    ``stale_lock`` (locked for some other request). ``stop`` leaves the last
    file behind, like a link service stopped by the shutdown."""

    STALE_ID = "00000000-1"

    def __init__(self, idle_file: Path, req_file: Path, mode: str) -> None:
        self.idle_file = idle_file
        self.req_file = req_file
        self.mode = mode
        self.seq = 0
        self.requests: list[str] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def _line(self) -> str:
        try:
            req = self.req_file.read_text(encoding="utf-8").split()[0]
        except (OSError, IndexError):
            req = ""
        if req and req not in self.requests:
            self.requests.append(req)
        if self.mode == "grant":
            return f"locked {self.seq} {req}" if req else f"idle {self.seq}"
        if self.mode == "stale_lock":
            return f"locked {self.seq} {self.STALE_ID}"
        return f"{self.mode} {self.seq}"

    def write_once(self) -> None:
        self.seq += 1
        tmp = self.idle_file.with_name(self.idle_file.name + ".tmp")
        tmp.write_text(self._line() + "\n", encoding="utf-8")
        os.replace(tmp, self.idle_file)

    def _loop(self) -> None:
        while not self._stop.wait(0.003):
            self.write_once()

    def start(self) -> "_FakeLink":
        self.idle_file.parent.mkdir(parents=True, exist_ok=True)
        self.write_once()
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=10)


def _code_lines(text: str) -> list[str]:
    """Script lines that are not comments (heredoc-free script)."""
    return [ln for ln in text.splitlines() if not ln.lstrip().startswith("#")]


# --------------------------------------------------------------------- static


class X8SUP_A_Static(unittest.TestCase):
    def setUp(self) -> None:
        self.text = _SCRIPT.read_text(encoding="utf-8")

    def test_files_exist(self) -> None:
        for p in (_SCRIPT, _UNIT, _CONF_EXAMPLE, _X8R / "README.md"):
            self.assertTrue(p.is_file(), p)

    def test_shebang_lf_and_exec_bit(self) -> None:
        raw = _SCRIPT.read_bytes()
        self.assertTrue(raw.startswith(b"#!/bin/sh\n"))
        self.assertNotIn(b"\r", raw)
        self.assertNotIn(b"\r", _UNIT.read_bytes())
        # supervisor.conf.example is not in .gitattributes' LF list, so a
        # Windows checkout may hold CRLF; the script tolerates that (tested
        # below) instead of this test failing on the bench laptop.
        if os.name == "posix":
            self.assertTrue(os.access(_SCRIPT, os.X_OK), "chmod +x lifetrac-supervisor.sh")

    @unittest.skipUnless(_HAVE_POSIX, "needs a POSIX sh")
    def test_syntax_under_every_available_shell(self) -> None:
        for sh in ("sh", "dash", "bash"):
            exe = shutil.which(sh)
            if exe is None:
                continue
            with self.subTest(shell=sh):
                cp = subprocess.run([exe, "-n", str(_SCRIPT)], capture_output=True, text=True)
                self.assertEqual(cp.returncode, 0, cp.stderr)

    def test_shellcheck_posix_clean_when_installed(self) -> None:
        exe = shutil.which("shellcheck")
        if exe is None:
            self.skipTest("shellcheck not installed")
        cp = subprocess.run([exe, "-s", "sh", str(_SCRIPT)], capture_output=True, text=True)
        self.assertEqual(cp.returncode, 0, cp.stdout + cp.stderr)

    def test_never_do_list_holds_in_code(self) -> None:
        code = "\n".join(_code_lines(self.text))
        forbidden = [
            r"gpio",
            r"openocd",
            r"\b(rmmod|insmod|modprobe)\b",
            r"x8h7",
            r"ttymxc",
            r"stm32h7-program",
            r"docker\.service",
            r"x8_max_carrier_recovery",
            r"\b(flash_l072|AN3155|unprotect)",
            r"python",
        ]
        for pat in forbidden:
            with self.subTest(pattern=pat):
                self.assertIsNone(re.search(pat, code, re.IGNORECASE), pat)
        # sysrq: only s, u and b, each as its own write; never c.
        writes = re.findall(r"^\s*sysrq_write\s+(\S+)", code, re.MULTILINE)
        self.assertEqual(writes, ["s", "u", "b"])
        self.assertIsNone(re.search(r"""(echo|printf)[^\n]*\bc\b[^\n]*sysrq""", code, re.IGNORECASE))
        # restart only ever names the loop variable fed from CONTAINERS.
        self.assertEqual(
            re.findall(r"docker restart (\S+)", code), ['"$_ct"']
        )

    def test_every_external_board_command_runs_under_timeout(self) -> None:
        # Command position: line start, after ; & | ( ` ! or $(, or after a
        # keyword, optionally negated -- so `if systemctl ...`, `elif docker
        # ...`, `! systemctl ...` and backticks count too (2026-10-17 review).
        cmd = re.compile(
            r"(?:^|[;&|(`!]|\$\(|\b(?:then|do|else|if|elif|while|until)\b)\s*(?:!\s*)?"
            r"(docker|systemctl|fw_printenv|systemd-notify|sync|stat)\b"
        )
        for bare_form in ("  if systemctl reboot >/dev/null 2>&1; then",
                          '  if ! docker restart "$_ct"; then',
                          "  elif docker info; then",
                          "  while docker info; do :; done",
                          "  ! systemctl reboot -ff",
                          "  _x=`docker info`"):
            self.assertTrue(cmd.search(bare_form), bare_form)
        for wrapped in ("  if timeout 20 docker info; then",
                        '  if ! timeout 60 docker restart "$_ct"; then',
                        "  until timeout 20 docker info; do :; done"):
            self.assertFalse(cmd.search(wrapped), wrapped)
        bare = [ln.strip() for ln in _code_lines(self.text) if cmd.search(ln)]
        self.assertEqual(bare, [], "external command without timeout")
        self.assertIn("timeout 20 docker info", self.text)
        self.assertIn("timeout 20 docker inspect", self.text)
        self.assertIn("timeout 60 docker restart", self.text)
        self.assertIn("timeout 30 systemctl reboot", self.text)
        self.assertIn("timeout 10 systemd-notify WATCHDOG=1", self.text)
        self.assertNotIn("--no-block", "\n".join(_code_lines(self.text)))

    def test_no_wall_clock_in_code(self) -> None:
        code = "\n".join(_code_lines(self.text))
        self.assertIsNone(re.search(r"\bdate\b", code))
        self.assertNotIn("%Y %s", code)

    def test_unit_file(self) -> None:
        unit = _UNIT.read_text(encoding="utf-8")
        body = [ln.strip() for ln in _code_lines(unit) if ln.strip()]
        for want in (
            "Type=simple",
            "Restart=on-failure",
            "RestartSec=10",
            "WatchdogSec=300",
            "NotifyAccess=all",
            "ExecStart=/opt/lifetrac/x8_recovery/lifetrac-supervisor.sh run",
        ):
            self.assertIn(want, body)
        # No docker.service dependency: Requires= would stop the supervisor
        # with docker, Wants= would start a stopped docker (and its radio
        # containers) on every start of this unit, and After= would block the
        # supervisor behind a dockerd that hangs before READY=1 (TimeoutSec=0).
        self.assertFalse([ln for ln in body if ln.startswith(("Requires=", "Wants=", "After=", "BindsTo=", "Requisite="))
                          and "docker" in ln])
        header = unit.split("[Unit]")[0]
        self.assertIn("/etc/systemd/system", header)
        self.assertIn("supervisor.armed", header)
        self.assertRegex(header, r"(?i)not installed by any script")
        # The header has to say why there is no docker dependency.
        self.assertRegex(header, r"(?is)no Requires=, Wants= or\s+#\s+After=")
        self.assertRegex(header, r"(?is)Wants= would start a stopped docker\.service")
        self.assertRegex(header, r"(?is)TimeoutSec=0")

    def test_conf_example_ships_empty_allowlist_with_warnings(self) -> None:
        text = _CONF_EXAMPLE.read_text(encoding="utf-8")
        active = [ln for ln in _code_lines(text) if ln.strip()]
        self.assertIn('CONTAINERS=""', [ln.strip() for ln in active])
        self.assertIn("TRACTOR_REBOOT_ENABLED=0", [ln.strip() for ln in active])
        # A verbatim copy must be tractor-safe; the base edits it to base.
        self.assertEqual([ln.strip() for ln in active if ln.startswith("ROLE")], ["ROLE=tractor"])
        # No LifeTrac container is safe on the allowlist yet (2026-10-17
        # review), so none may be listed or one uncomment away: a broker
        # restart drops the subscriptions of clients without on_connect
        # (lora_bridge's cmd/estop), web_ui republishes retained radio
        # settings, the image daemons send the L072 a RESET_REQ.
        for ln in text.splitlines():
            body = ln.strip().lstrip("#").strip()
            if body.startswith("CONTAINERS="):
                for name in ("mosquitto", "web_ui", "lora_bridge", "image_"):
                    self.assertNotIn(name, body, ln)
        self.assertRegex(text, r"(?is)mosquitto.*on_connect")
        self.assertRegex(text, r"(?is)web_ui.*retained")
        self.assertIn("RESET_REQ", text)
        # D-PWR1: the example names the idle-only rule next to the switch.
        self.assertIn("TRACTOR_IDLE_WAIT_S=10", [ln.strip() for ln in active])
        self.assertRegex(text, r"(?s)D-PWR1.*TRACTOR_REBOOT_ENABLED=0")

    def test_supervisor_only_reads_the_h747_idle_file(self) -> None:
        # D-PWR1: the idle report comes from the H747's link service; the
        # supervisor writing or deleting it could fake an idle tractor.
        code = "\n".join(_code_lines(self.text))
        self.assertIsNone(re.search(r'>>?\s*"?\$IDLE_FILE', code))
        for verb in ("kv_put", "write_record", "rm -f", "mv -f"):
            self.assertNotIn(f'{verb} "$IDLE_FILE"', code)
        self.assertIn('kv_put "$REQ_FILE"', code)


# ------------------------------------------------------------------ behaviour


@unittest.skipUnless(_HAVE_POSIX, "needs a POSIX sh and coreutils timeout")
class X8SUP_B_Supervisor(unittest.TestCase):
    SHELL = "sh"

    # ------------------------------------------------------------- fixture
    def setUp(self) -> None:
        exe = shutil.which(self.SHELL)
        if exe is None:
            self.skipTest(f"{self.SHELL} not installed")
        self.shell = exe
        self._tmp = tempfile.TemporaryDirectory(prefix="lt_x8sup_")
        self.addCleanup(self._tmp.cleanup)
        root = Path(self._tmp.name)
        self.root = root
        self.bin = root / "bin"
        self.tools = root / "tools"
        self.fake = root / "fake"
        self.state = root / "state"
        self.etc = root / "etc"
        for d in (self.bin, self.tools, self.fake / "containers", self.etc):
            d.mkdir(parents=True)
        for tool in _REAL_TOOLS:
            real = shutil.which(tool)
            if real is None:
                self.skipTest(f"{tool} not installed")
            (self.tools / tool).symlink_to(real)
        for name, body in (
            ("docker", _FAKE_DOCKER),
            ("systemctl", _FAKE_SYSTEMCTL),
            ("fw_printenv", _FAKE_FW_PRINTENV),
            ("systemd-notify", _FAKE_NOTIFY),
            ("sync", _FAKE_SYNC),
        ):
            p = self.bin / name
            p.write_text(body, encoding="utf-8")
            p.chmod(0o755)
        self.fake_log = root / "fake.log"
        self.fake_log.write_text("", encoding="utf-8")
        self.uptime_file = root / "uptime"
        self.boot_file = root / "boot_id"
        self.sysrq = root / "sysrq-trigger"
        self.armed = self.etc / "supervisor.armed"
        self.conf = self.etc / "supervisor.conf"
        self.idle_file = root / "run" / "tractor_idle"
        self.req_file = root / "run" / "tractor_reboot_request"
        self.set_uptime(1000)
        self.set_boot(BOOT_A)
        self.write_conf()

    # ------------------------------------------------------------- helpers
    def set_uptime(self, seconds: int) -> None:
        self.uptime_file.write_text(f"{seconds}.42 {seconds * 3}.17\n", encoding="utf-8")

    def set_boot(self, boot_id: str) -> None:
        self.boot_file.write_text(boot_id + "\n", encoding="utf-8")

    def write_conf(self, **overrides: str) -> None:
        conf = dict(_DEFAULT_CONF)
        conf.update(overrides)
        lines = [f'{k}="{v}"' for k, v in conf.items()]
        self.conf.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def arm(self) -> None:
        self.armed.write_text("GO\n", encoding="utf-8")

    def set_container(self, name: str, state: str, resolves: str | None = None) -> None:
        """``state`` is "<State.Status> <Health>"; ``resolves`` fakes docker
        answering for another container (an ID-prefix match)."""
        (self.fake / "containers" / name).write_text(state + "\n", encoding="utf-8")
        if resolves is not None:
            (self.fake / "containers" / f"{name}.resolves").write_text(resolves + "\n", encoding="utf-8")

    def fresh(self) -> None:
        """Forget all supervisor state and fake calls (between sub-cases)."""
        shutil.rmtree(self.state, ignore_errors=True)
        self.fake_log.write_text("", encoding="utf-8")

    def pending_marker(self) -> Path:
        return self.state / "per_boot" / "reboot_pending"

    def counter(self) -> str:
        return (self.state_text("consecutive_reboots") or "").strip()

    def set_docker_info_rc(self, rc: int) -> None:
        (self.fake / "docker_info_rc").write_text(f"{rc}\n", encoding="utf-8")

    def set_fake(self, name: str, value: str) -> None:
        (self.fake / name).write_text(value + "\n", encoding="utf-8")

    def put_state(self, name: str, value: str) -> None:
        self.state.mkdir(parents=True, exist_ok=True)
        (self.state / name).write_text(value + "\n", encoding="utf-8")

    def state_text(self, name: str) -> str | None:
        p = self.state / name
        return p.read_text(encoding="utf-8") if p.exists() else None

    def state_kv(self, name: str) -> dict[str, str]:
        text = self.state_text(name) or ""
        out = {}
        for ln in text.splitlines():
            if "=" in ln:
                k, v = ln.split("=", 1)
                out[k] = v
        return out

    def fake_lines(self) -> list[str]:
        return self.fake_log.read_text(encoding="utf-8").splitlines()

    def calls(self, prefix: str) -> list[str]:
        return [ln for ln in self.fake_lines() if ln.startswith(prefix)]

    def sv_log(self) -> str:
        return self.state_text("supervisor.log") or ""

    def sysrq_text(self) -> str:
        return self.sysrq.read_text(encoding="utf-8") if self.sysrq.exists() else ""

    def env(self, extra: dict[str, str] | None = None) -> dict[str, str]:
        env = {
            "PATH": f"{self.bin}{os.pathsep}{self.tools}",
            "LC_ALL": "C",
            "LIFETRAC_SUPERVISOR_CONF": str(self.conf),
            "LIFETRAC_SUPERVISOR_STATE": str(self.state),
            "LIFETRAC_SUPERVISOR_ARMED_FILE": str(self.armed),
            "LIFETRAC_SYSRQ": str(self.sysrq),
            "LIFETRAC_PROC_UPTIME": str(self.uptime_file),
            "LIFETRAC_BOOT_ID_FILE": str(self.boot_file),
            "FAKE_LOG": str(self.fake_log),
            "FAKE_DIR": str(self.fake),
            "FAKE_STATE": str(self.state),
            "FAKE_UPTIME": str(self.uptime_file),
            "FAKE_REQ": str(self.req_file),
            "LIFETRAC_TRACTOR_IDLE_FILE": str(self.idle_file),
            "LIFETRAC_TRACTOR_REBOOT_REQUEST": str(self.req_file),
            "LIFETRAC_IDLE_POLL": "0.02",
        }
        if extra:
            env.update(extra)
        return env

    def run_sv(self, *args: str, extra: dict[str, str] | None = None) -> "subprocess.CompletedProcess[str]":
        cp = subprocess.run(
            [self.shell, str(_SCRIPT), *(args or ("once",))],
            env=self.env(extra),
            capture_output=True,
            text=True,
            timeout=60,
        )
        self.assertEqual(cp.returncode, 0, f"stdout:\n{cp.stdout}\nstderr:\n{cp.stderr}")
        self.assertEqual(cp.stderr, "", cp.stderr)
        return cp

    def passes(self, n: int, start: int | None = None, step: int = 30) -> None:
        """Run n passes, optionally advancing uptime from ``start`` by ``step``."""
        for i in range(n):
            if start is not None:
                self.set_uptime(start + i * step)
            self.run_sv("once")

    def start_link(self, mode: str = "grant") -> _FakeLink:
        link = _FakeLink(self.idle_file, self.req_file, mode).start()
        self.addCleanup(link.stop)
        return link

    def write_idle(self, line: str) -> None:
        """An idle file nobody updates (a dead or missing link service)."""
        self.idle_file.parent.mkdir(parents=True, exist_ok=True)
        self.idle_file.write_text(line + "\n", encoding="utf-8")

    def request(self) -> str | None:
        return self.req_file.read_text(encoding="utf-8").strip() if self.req_file.exists() else None

    def fake_kv(self, name: str) -> dict[str, str]:
        p = self.fake / name
        text = p.read_text(encoding="utf-8") if p.exists() else ""
        return dict(ln.split("=", 1) for ln in text.splitlines() if "=" in ln)

    # `run` loop (what lifetrac-supervisor.service runs). Own process group,
    # so stopping it also stops a `sleep` or `timeout` child; cleanup kills
    # whatever is left before the temp dir goes.
    def start_run(self) -> None:
        self.run_err = self.root / "run.stderr"
        with open(self.root / "run.stdout", "w", encoding="utf-8") as out, \
                open(self.run_err, "w", encoding="utf-8") as err:
            self.run_proc = subprocess.Popen(
                [self.shell, str(_SCRIPT), "run"], env=self.env(),
                stdout=out, stderr=err, start_new_session=True)
        self.addCleanup(self._kill_run)

    def _kill_run(self) -> None:
        if self.run_proc.poll() is None:
            try:
                os.killpg(self.run_proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.run_proc.wait(timeout=10)

    def stop_run(self) -> None:
        self.assertIsNone(self.run_proc.poll(), "the run loop exited by itself")
        os.killpg(self.run_proc.pid, signal.SIGTERM)
        self.run_proc.wait(timeout=10)
        self.assertEqual(self.run_err.read_text(encoding="utf-8"), "")

    def wait_for(self, cond, what: str, timeout: float = 20.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cond():
                return
            if self.run_proc.poll() is not None:
                self.fail(f"run loop exited (rc={self.run_proc.returncode}) waiting for {what}: "
                          f"{self.run_err.read_text(encoding='utf-8')}")
            time.sleep(0.05)
        self.fail(f"timed out waiting for {what}; supervisor.log:\n{self.sv_log()}")

    # --------------------------------------------------------------- tests
    def test_disarmed_logs_would_and_never_acts(self) -> None:
        self.write_conf(CONTAINERS="web_ui", REBOOT_GRACE_S="0")
        self.set_container("web_ui", "running unhealthy")
        self.passes(3)
        self.assertIn("WOULD restart container=web_ui", self.sv_log())
        self.set_docker_info_rc(1)
        self.passes(3)
        log = self.sv_log()
        self.assertIn("WOULD reboot reason=docker_down", log)
        self.assertNotIn("ACTION", log)
        self.assertEqual(self.calls("docker restart"), [])
        self.assertEqual(self.calls("systemctl"), [])
        self.assertEqual(self.sysrq_text(), "")
        for name in ("last_action", "held", "consecutive_reboots"):
            self.assertIsNone(self.state_text(name), name)
        self.assertEqual(self.state_kv("status")["armed"], "0")

    def test_disarmed_dry_run_walks_the_whole_ladder(self) -> None:
        self.write_conf(CONTAINERS="web_ui", FAIL_THRESHOLD="1",
                        CONTAINER_RESTART_MAX="1", REBOOT_GRACE_S="0")
        self.set_container("web_ui", "exited none")
        self.passes(2)
        log = self.sv_log()
        self.assertIn("WOULD restart container=web_ui", log)
        self.assertIn("WOULD reboot reason=container_exhausted:web_ui", log)
        self.assertEqual(self.calls("docker restart"), [])
        self.assertEqual(self.calls("systemctl"), [])
        # Arming afterwards starts from a full, real restart budget.
        self.arm()
        self.passes(1)
        self.assertEqual(self.calls("docker restart"), ["docker restart web_ui"])

    def test_armed_restarts_only_allowlisted_container_after_threshold(self) -> None:
        self.arm()
        self.write_conf(CONTAINERS="web_ui")
        self.set_container("web_ui", "running unhealthy")
        self.set_container("rogue", "exited none")
        self.passes(2)
        self.assertEqual(self.calls("docker restart"), [])
        self.passes(1)
        self.assertEqual(self.calls("docker restart"), ["docker restart web_ui"])
        inspects = self.calls("docker inspect")
        self.assertTrue(inspects)
        for ln in inspects:
            self.assertIn("--type container", ln)
            self.assertIn("{{.State.Status}}", ln)
            self.assertIn(".State.Health.Status", ln)
            self.assertTrue(ln.endswith(" web_ui"), ln)
        self.assertNotIn("rogue", "\n".join(self.fake_lines()))
        # Streak resets after a restart: two more bad passes do nothing.
        self.passes(2)
        self.assertEqual(len(self.calls("docker restart")), 1)
        self.passes(1)
        self.assertEqual(len(self.calls("docker restart")), 2)
        self.assertEqual(self.calls("systemctl"), [])

    def test_container_state_classification(self) -> None:
        self.arm()
        self.write_conf(CONTAINERS="ctr", FAIL_THRESHOLD="1")
        cases = {
            "running healthy": False,
            "running none": False,
            "running starting": False,
            "running unhealthy": True,
            "exited none": True,
            "restarting none": True,
            "paused healthy": True,
            None: True,  # missing container
        }
        for i, (state, restarts) in enumerate(cases.items()):
            with self.subTest(state=state):
                self.set_boot(f"dddddddd-0000-4000-8000-{i:012d}")  # fresh budget each
                p = self.fake / "containers" / "ctr"
                if state is None:
                    if p.exists():
                        p.unlink()
                else:
                    self.set_container("ctr", state)
                before = len(self.calls("docker restart"))
                self.run_sv("once")
                after = len(self.calls("docker restart"))
                self.assertEqual(after - before, 1 if restarts else 0)
        self.assertIn("ACTION restart container=ctr reason=missing", self.sv_log())

    def test_restart_cap_per_boot_then_reboot_escalation(self) -> None:
        self.arm()
        self.write_conf(CONTAINERS="web_ui", FAIL_THRESHOLD="2",
                        CONTAINER_RESTART_MAX="2", REBOOT_GRACE_S="0")
        self.set_container("web_ui", "running unhealthy")
        self.passes(4)
        self.assertEqual(len(self.calls("docker restart")), 2)
        self.assertEqual(self.calls("systemctl"), [])
        self.passes(1)
        self.assertIn("restarts used up (2/2 this boot) -> node-level fault", self.sv_log())
        self.assertEqual(self.calls("systemctl"), [])
        self.passes(1)
        self.assertEqual(len(self.calls("docker restart")), 2)
        self.assertEqual(self.calls("systemctl reboot"), ["systemctl reboot [last_action=reboot consecutive=1]"])
        self.assertEqual(self.state_kv("last_action")["reason"], "container_exhausted:web_ui")
        # The restart budget is per boot (keyed by boot_id).
        self.set_boot(BOOT_B)
        self.passes(2)
        self.assertEqual(len(self.calls("docker restart")), 3)

    def test_restart_refused_when_its_count_cannot_be_recorded(self) -> None:
        # Copilot review on PR #153: on full or read-only storage the restart
        # count stays at its old value, so every pass would restart again past
        # CONTAINER_RESTART_MAX. A directory where the counter's temp file
        # goes makes the write fail even for root.
        self.arm()
        self.write_conf(CONTAINERS="ctr", FAIL_THRESHOLD="1",
                        CONTAINER_RESTART_MAX="1", REBOOT_GRACE_S="0")
        self.set_container("ctr", "running healthy")
        self.run_sv("once")  # sets up this boot's per-boot state
        (self.state / "per_boot" / "c.ctr.restarts.tmp").mkdir()
        self.set_container("ctr", "running unhealthy")
        self.run_sv("once")
        self.assertEqual(self.calls("docker restart"), [])
        self.assertIn("REFUSED restart container=ctr: cannot record restart 1/1", self.sv_log())
        self.assertIn("restart_unrecorded:ctr", self.state_kv("status")["decision"])
        # Escalates like an exhausted budget: a node-level fault, so the
        # reboot rung runs (its own counter is still writable here).
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertEqual(self.state_kv("last_action")["reason"], "container_unrecorded:ctr")
        # Later passes in the same boot still restart nothing.
        self.passes(2)
        self.assertEqual(self.calls("docker restart"), [])

    def test_grace_blocks_reboot_until_uptime(self) -> None:
        self.arm()
        self.write_conf()
        self.set_docker_info_rc(1)
        self.passes(3, start=100)
        log = self.sv_log()
        self.assertIn("BLOCKED reboot gate=grace up=160s<600s reason=docker_down", log)
        self.assertEqual(self.calls("systemctl"), [])
        self.assertEqual(self.state_kv("status")["reboot_blocked"], "grace")
        self.set_uptime(600)
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)

    def test_tractor_gate_blocks_reboot_unless_enabled(self) -> None:
        self.arm()
        self.write_conf(ROLE="tractor")
        self.set_docker_info_rc(1)
        self.passes(3)
        self.assertIn("BLOCKED reboot gate=tractor_reboot_disabled", self.sv_log())
        status = self.state_kv("status")
        self.assertEqual(status["reboot_blocked"], "tractor_reboot_disabled")
        self.assertIn("reboot_blocked:tractor_reboot_disabled", status["decision"])
        self.assertEqual(status["role"], "tractor")
        self.assertEqual(self.calls("systemctl"), [])
        self.assertIsNone(self.state_text("last_action"))
        self.assertIsNone(self.state_text("consecutive_reboots"))
        self.write_conf(ROLE="tractor", TRACTOR_REBOOT_ENABLED="1")
        self.start_link("grant")
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)

    def test_reboot_writes_counter_and_last_action_before_systemctl(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0")
        self.set_docker_info_rc(1)
        self.run_sv("once")
        lines = self.fake_lines()
        idx = next(i for i, ln in enumerate(lines) if ln.startswith("systemctl reboot"))
        # The fake systemctl reads both records at the moment it is called:
        # the boot-loop counter and the reset reason are already on disk.
        self.assertEqual(lines[idx], "systemctl reboot [last_action=reboot consecutive=1]")
        # What the rung did between the last health check and the call. Each
        # record is synced while still a tmp file (data before the rename),
        # then again after the rename. boot_init's own syncs come before the
        # health checks, so they cannot satisfy this.
        last_docker = max(i for i, ln in enumerate(lines[:idx]) if ln.startswith("docker "))
        rung = lines[last_docker + 1:idx]
        c = rung.index("sync [tmp=consecutive_reboots.tmp]")
        a = rung.index("sync [tmp=last_action.tmp]")
        self.assertLess(c, a, "the counter must be persisted before last_action")
        self.assertEqual(rung[c + 1], "sync [tmp=]")
        self.assertEqual(rung[a + 1], "sync [tmp=]")
        self.assertEqual(rung[-1], "sync [tmp=]")
        la = self.state_kv("last_action")
        self.assertEqual(la["action"], "reboot")
        self.assertEqual(la["reason"], "docker_down")
        self.assertEqual(la["boot_id"], BOOT_A)
        self.assertEqual(la["uptime_s"], "1000")
        self.assertEqual(la["consecutive"], "1")
        self.assertEqual((self.state_text("consecutive_reboots") or "").strip(), "1")
        self.assertFalse(list(self.state.glob("*.tmp")))
        log = self.sv_log()
        self.assertLess(log.index("ACTION reboot reason=docker_down"),
                        log.index("systemctl reboot accepted"))
        self.assertRegex(log, r"boot=aaaaaaaa up=1000 ACTION reboot")

    def test_consecutive_cap_enters_held_and_stops_rebooting(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0", REBOOT_MAX_CONSECUTIVE="2")
        self.set_docker_info_rc(1)
        for boot in (BOOT_A, BOOT_B):
            self.set_boot(boot)
            self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 2)
        self.set_boot(BOOT_C)
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 2)
        held = self.state_kv("held")
        self.assertEqual(held["reason"], "docker_down")
        self.assertEqual(held["consecutive"], "2")
        self.assertEqual(held["cap"], "2")
        self.assertIn("HELD entering hold", self.sv_log())
        self.set_uptime(1030)
        cp = self.run_sv("once")
        self.assertIn("HELD: supervisor reboots suppressed", cp.stdout)
        self.assertIn("HELD reboot suppressed reason=docker_down", cp.stdout)
        self.assertEqual(len(self.calls("systemctl reboot")), 2)
        self.assertEqual(self.state_kv("status")["held"], "1")
        # Container restarts still happen while held.
        self.set_docker_info_rc(0)
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0",
                        REBOOT_MAX_CONSECUTIVE="2", CONTAINERS="mosq")
        self.set_container("mosq", "running unhealthy")
        self.run_sv("once")
        self.assertEqual(self.calls("docker restart"), ["docker restart mosq"])

    def test_disarmed_at_the_cap_logs_would_hold_and_writes_no_hold(self) -> None:
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0", REBOOT_MAX_CONSECUTIVE="2")
        self.put_state("consecutive_reboots", "2")
        self.set_docker_info_rc(1)
        self.run_sv("once")
        self.assertIn("WOULD hold (disarmed): consecutive_reboots=2 reached cap=2 reason=docker_down",
                      self.sv_log())
        self.assertIsNone(self.state_text("held"))
        status = self.state_kv("status")
        self.assertEqual(status["held"], "0")
        self.assertIn("would_hold", status["decision"])
        self.assertEqual(self.calls("systemctl"), [])
        self.assertEqual(self.counter(), "2")

    def test_run_loop_disarm_takes_effect_on_the_next_pass(self) -> None:
        # README: "Remove the file to disarm; the next pass logs the change".
        # Only the service's `run` loop re-reads it from pass to pass; `once`
        # starts disarmed in every process, so it cannot show this.
        self.arm()
        self.write_conf(CONTAINERS="mosq", FAIL_THRESHOLD="1",
                        CONTAINER_RESTART_MAX="100", CHECK_INTERVAL_S="1")
        self.set_container("mosq", "running unhealthy")
        self.start_run()
        self.wait_for(lambda: len(self.calls("docker restart")) >= 2, "two armed restarts")
        self.armed.unlink()
        mark = "ARMED state 1 -> 0"
        self.wait_for(lambda: mark in self.sv_log(), "the disarm log line")
        self.wait_for(lambda: self.sv_log().split(mark, 1)[1].count("WOULD restart") >= 2,
                      "two disarmed passes")
        self.stop_run()
        log = self.sv_log()
        # A pass reads the armed file once, at its start: nothing after the
        # line that logs the change acts.
        self.assertNotIn("ACTION", log.split(mark, 1)[1])
        self.assertEqual(len(self.calls("docker restart")),
                         log.count("ACTION restart container=mosq reason="))
        self.assertEqual(self.calls("systemctl"), [])
        self.assertEqual(self.state_kv("status")["armed"], "0")

    def test_run_loop_clear_hold_takes_effect_on_the_next_pass(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0", CHECK_INTERVAL_S="1")
        self.put_state("held", "reason=docker_down\nconsecutive=3\ncap=3")
        self.put_state("consecutive_reboots", "3")
        self.set_docker_info_rc(1)
        self.start_run()
        self.wait_for(lambda: "HELD reboot suppressed" in self.sv_log(), "a held pass")
        self.assertEqual(self.calls("systemctl"), [])
        self.run_sv("clear-hold")
        self.wait_for(lambda: self.calls("systemctl reboot"), "the reboot after clear-hold")
        self.wait_for(lambda: self.state_kv("status").get("decision") == "reboot_pending",
                      "a pass that waits on the accepted reboot")
        self.stop_run()
        self.assertEqual(self.calls("systemctl reboot"),
                         ["systemctl reboot [last_action=reboot consecutive=1]"])
        self.assertEqual(self.state_kv("status")["held"], "0")

    def test_clear_hold(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0", REBOOT_MAX_CONSECUTIVE="3")
        self.put_state("held", "reason=docker_down\nconsecutive=3\ncap=3")
        self.put_state("consecutive_reboots", "3")
        self.set_docker_info_rc(1)
        self.run_sv("once")
        self.assertEqual(self.calls("systemctl"), [])
        self.run_sv("clear-hold")
        self.assertIsNone(self.state_text("held"))
        self.assertEqual((self.state_text("consecutive_reboots") or "").strip(), "0")
        self.assertIn("CLEAR-HOLD by operator", self.sv_log())
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertEqual((self.state_text("consecutive_reboots") or "").strip(), "1")

    def test_ota_probation_raises_cap_to_bootlimit_plus_one(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0", REBOOT_MAX_CONSECUTIVE="2")
        self.set_docker_info_rc(1)
        self.put_state("consecutive_reboots", "2")
        self.set_fake("fw_upgrade_available", "1")
        self.set_fake("fw_bootlimit", "4")
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertIn("consecutive-reboot cap = max(2, 4+1) = 5", self.sv_log())
        self.assertIn("fw_printenv -n upgrade_available", self.calls("fw_printenv"))
        self.assertEqual((self.state_text("consecutive_reboots") or "").strip(), "3")
        self.set_boot(BOOT_B)
        self.put_state("consecutive_reboots", "5")
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertEqual(self.state_kv("held")["cap"], "5")

    def test_probation_default_bootlimit_and_off_cases(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0", REBOOT_MAX_CONSECUTIVE="1")
        self.set_docker_info_rc(1)
        # bootlimit unset -> 5 -> cap 6
        self.put_state("consecutive_reboots", "5")
        self.set_fake("fw_upgrade_available", "1")
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertIn("bootlimit=5", self.sv_log())
        # Not on probation -> cap is REBOOT_MAX_CONSECUTIVE.
        self.set_boot(BOOT_B)
        self.put_state("consecutive_reboots", "1")
        self.set_fake("fw_upgrade_available", "0")
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertEqual(self.state_kv("held")["cap"], "1")
        # fw_printenv missing entirely -> treated as not on probation.
        (self.bin / "fw_printenv").unlink()
        self.run_sv("clear-hold")
        self.put_state("consecutive_reboots", "1")
        self.set_boot(BOOT_C)
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertEqual(self.state_kv("held")["cap"], "1")

    def test_healthy_streak_resets_counter(self) -> None:
        self.write_conf(HEALTHY_RESET_S="1800")
        self.put_state("consecutive_reboots", "2")
        self.set_uptime(1000)
        self.run_sv("once")
        self.set_uptime(2000)
        self.set_docker_info_rc(1)  # one bad pass restarts the healthy clock
        self.run_sv("once")
        self.set_docker_info_rc(0)
        self.set_uptime(2030)
        self.run_sv("once")
        self.set_uptime(3829)
        self.run_sv("once")
        self.assertEqual((self.state_text("consecutive_reboots") or "").strip(), "2")
        self.set_uptime(3830)
        self.run_sv("once")
        self.assertEqual((self.state_text("consecutive_reboots") or "").strip(), "0")
        self.assertIn("HEALTHY for 1800s of uptime: consecutive_reboots 2 -> 0", self.sv_log())

    def test_next_boot_reset_reason_classification(self) -> None:
        self.run_sv("once")
        self.assertEqual((self.state_text("last_reset_reason") or "").strip(), "external_or_unknown")
        self.assertEqual((self.state_text("boot_id") or "").strip(), BOOT_A)
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0")
        self.set_docker_info_rc(1)
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        # Supervisor-initiated reboot: the next boot says so.
        self.set_boot(BOOT_B)
        self.set_docker_info_rc(0)
        self.set_uptime(40)
        self.run_sv("once")
        want = "supervisor:reboot:docker_down"
        self.assertEqual((self.state_text("last_reset_reason") or "").strip(), want)
        # A restart of the supervisor inside the same boot does not reclassify.
        self.set_uptime(70)
        self.run_sv("once")
        self.assertEqual((self.state_text("last_reset_reason") or "").strip(), want)
        # Boot B ended without a supervisor action -> external.
        self.set_boot(BOOT_C)
        self.run_sv("once")
        self.assertEqual((self.state_text("last_reset_reason") or "").strip(), "external_or_unknown")
        self.assertEqual((self.state_text("boot_id") or "").strip(), BOOT_C)

    def test_systemctl_failure_forces_sysrq_s_u_b(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0")
        self.set_docker_info_rc(1)
        self.set_fake("systemctl_reboot_rc", "1")
        self.sysrq.write_text("", encoding="utf-8")
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "sub")
        self.assertNotIn("c", self.sysrq_text())
        sysctl = self.calls("systemctl")
        self.assertEqual(sysctl[0], "systemctl reboot [last_action=reboot consecutive=1]")
        self.assertEqual(sysctl[-1], "systemctl reboot -ff [last_action=force_reboot consecutive=1]")
        # The record on disk when `reboot -ff` ran.
        la = self.fake_kv("last_action.at_reboot_ff")
        self.assertEqual(la["action"], "force_reboot")
        self.assertEqual(la["reason"], "docker_down")
        self.assertEqual(la["detail"], "systemctl_reboot_rc_1")
        self.assertEqual(la["consecutive"], "1")  # the forced path does not count twice
        # The fake returns, so this forced path came back: the record says so.
        la = self.state_kv("last_action")
        self.assertEqual(la["action"], "reboot_abandoned")
        self.assertEqual(la["detail"], "forced_path_returned:systemctl_reboot_rc_1")
        log = self.sv_log()
        self.assertLess(log.index("SYSRQ wrote 's'"), log.index("SYSRQ wrote 'u'"))
        self.assertLess(log.index("SYSRQ wrote 'u'"), log.index("SYSRQ wrote 'b'"))
        self.assertIn("ABANDONED reboot reason=docker_down", log)
        # A forced reboot that did happen: the next boot finds the record as
        # it was at the `reboot -ff` call.
        shutil.copyfile(self.fake / "last_action.at_reboot_ff", self.state / "last_action")
        self.set_boot(BOOT_B)
        self.set_docker_info_rc(0)
        self.run_sv("once")
        self.assertEqual((self.state_text("last_reset_reason") or "").strip(),
                         "supervisor:force_reboot:docker_down")

    def test_sysrq_writes_are_settle_seconds_apart(self) -> None:
        # The SIL fixture pins SYSRQ_SETTLE_S=0; this one sleeps for real:
        # s, 1 s, u, 1 s, b.
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0", SYSRQ_SETTLE_S="1")
        self.set_docker_info_rc(1)
        self.set_fake("systemctl_reboot_rc", "1")
        self.sysrq.write_text("", encoding="utf-8")
        t0 = time.monotonic()
        self.run_sv("once")
        self.assertGreaterEqual(time.monotonic() - t0, 1.9)
        self.assertEqual(self.sysrq_text(), "sub")

    def test_forced_path_without_sysrq_file_uses_reboot_ff(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0")
        self.set_docker_info_rc(1)
        self.set_fake("systemctl_reboot_rc", "124")
        self.run_sv("once")
        self.assertFalse(self.sysrq.exists(), "must not create the sysrq file")
        self.assertEqual(self.calls("systemctl reboot -ff"),
                         ["systemctl reboot -ff [last_action=force_reboot consecutive=1]"])
        self.assertIn("rc_124", self.state_kv("last_action")["detail"])

    def test_still_alive_after_accepted_reboot_forces_after_timeout(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0", FORCE_REBOOT_AFTER_S="180")
        self.set_docker_info_rc(1)
        self.sysrq.write_text("", encoding="utf-8")
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.set_uptime(1179)
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)  # no second reboot
        self.assertIn("PENDING reboot requested 179s ago", self.sv_log())
        self.set_uptime(1180)
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "sub")
        self.assertEqual((self.state_text("consecutive_reboots") or "").strip(), "1")
        self.assertIn("alive_180s_after_systemctl_reboot", self.state_kv("last_action")["detail"])
        # One forced attempt only; if still alive, the counted ladder resumes.
        self.set_uptime(1210)
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "sub")
        self.assertEqual(len(self.calls("systemctl reboot")), 3)  # reboot, -ff, reboot
        self.assertEqual((self.state_text("consecutive_reboots") or "").strip(), "2")

    # The forced path after an accepted reboot re-applies the gates (review
    # findings, 2026-10-17): it used to check ARMED only, kept its marker
    # while disarmed (stopping every health check for the rest of the boot)
    # and fired on re-arm hours later.
    def _accepted_reboot(self, **conf: str) -> None:
        """Armed, docker down, one pass at up=1000: `systemctl reboot` accepted."""
        self.arm()
        settings = {"FAIL_THRESHOLD": "1", "REBOOT_GRACE_S": "0"}
        settings.update(conf)
        self.write_conf(**settings)
        if settings.get("ROLE") == "tractor" and not hasattr(self, "link"):
            self.link = self.start_link("grant")
        self.set_docker_info_rc(1)
        self.sysrq.write_text("", encoding="utf-8")
        self.set_uptime(1000)
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertTrue(self.pending_marker().exists())

    def test_forced_path_rechecks_the_tractor_gate(self) -> None:
        self._accepted_reboot(ROLE="tractor", TRACTOR_REBOOT_ENABLED="1")
        # The operator closes the gate and restarts the service.
        self.write_conf(ROLE="tractor", TRACTOR_REBOOT_ENABLED="0",
                        FAIL_THRESHOLD="1", REBOOT_GRACE_S="0")
        self.set_uptime(1180)
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertIn("BLOCKED forced reboot gate=tractor_reboot_disabled", self.sv_log())
        self.assertFalse(self.pending_marker().exists())
        self.assertEqual(self.state_kv("status")["reboot_blocked"], "tractor_reboot_disabled")
        # The abandoned reboot releases the H747's motion hold.
        self.assertIsNone(self.request())
        self.assertIn("LOCK withdrawn (reboot abandoned)", self.sv_log())

    def test_forced_path_respects_a_hold_written_after_the_accept(self) -> None:
        self._accepted_reboot()
        self.put_state("held", "reason=operator\nconsecutive=1\ncap=3")
        self.set_uptime(1180)
        cp = self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "")
        self.assertIn("BLOCKED forced reboot gate=held", cp.stdout)
        self.assertNotIn("SYSRQ", cp.stdout)
        self.assertEqual(len(self.calls("systemctl reboot")), 1)

    def test_disarming_drops_the_pending_reboot_and_checks_continue(self) -> None:
        self._accepted_reboot()
        self.armed.unlink()
        for up in (1030, 1200, 5000):
            self.set_uptime(up)
            self.run_sv("once")
        log = self.sv_log()
        self.assertIn("BLOCKED forced reboot gate=disarmed", log)
        self.assertFalse(self.pending_marker().exists())
        self.assertEqual(len(self.calls("docker info")), 4)  # every pass checks
        self.assertIn("WOULD reboot reason=docker_down", log)
        # Re-arming hours later, node healthy: nothing is forced.
        self.arm()
        self.set_docker_info_rc(0)
        self.set_uptime(20000)
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "")
        self.assertEqual(len(self.calls("systemctl")), 1)

    def test_pending_reboot_is_not_forced_once_the_node_is_healthy(self) -> None:
        self._accepted_reboot()
        self.set_docker_info_rc(0)
        self.set_uptime(1180)
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "")
        self.assertEqual(len(self.calls("systemctl")), 1)
        self.assertIn("no node fault now: not forcing", self.sv_log())
        self.assertFalse(self.pending_marker().exists())

    def test_force_window_counts_from_the_accept_not_the_pass_start(self) -> None:
        # Review finding (2026-10-17): the marker held the pass-start uptime,
        # so a slow rung (syncs, fw_printenv, systemctl) ate the window, the
        # marker read as stale and a second counted reboot went out instead.
        self.set_fake("uptime_at_accept", "1150.00 3450.00")
        self._accepted_reboot(FORCE_REBOOT_AFTER_S="180", CHECK_INTERVAL_S="30")
        self.assertEqual(self.pending_marker().read_text(encoding="utf-8").split()[0], "1150")
        self.set_uptime(1329)
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "")
        self.assertIn("PENDING reboot requested 179s ago", self.sv_log())
        self.set_uptime(1330)
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "sub")
        self.assertNotIn("PENDING marker stale", self.sv_log())
        self.assertEqual(self.calls("systemctl reboot -ff"),
                         ["systemctl reboot -ff [last_action=force_reboot consecutive=1]"])

    def test_abandoned_reboot_is_not_read_as_a_supervisor_reboot_later(self) -> None:
        # Review finding (2026-10-17): after a pending reboot was dropped the
        # board stayed up with last_action naming this boot, so a power cut
        # hours later read as supervisor:reboot:<reason>.
        for case in ("healthy", "disarmed", "stale"):
            with self.subTest(case=case):
                self.fresh()
                self.set_boot(BOOT_A)
                self._accepted_reboot()
                if case == "healthy":
                    self.set_docker_info_rc(0)
                    up, want = 1180, "no node fault now: not forcing"
                elif case == "disarmed":
                    self.armed.unlink()
                    up, want = 1180, "BLOCKED forced reboot gate=disarmed"
                else:
                    self.set_docker_info_rc(0)
                    up, want = 1241, "PENDING marker stale"
                self.set_uptime(up)
                self.run_sv("once")
                self.assertIn(want, self.sv_log())
                la = self.state_kv("last_action")
                self.assertEqual((la["action"], la["reason"], la["boot_id"]),
                                 ("reboot_abandoned", "docker_down", BOOT_A))
                self.set_uptime(50000)
                self.run_sv("once")
                self.set_boot(BOOT_B)
                self.set_docker_info_rc(0)
                self.set_uptime(40)
                self.run_sv("once")
                self.assertEqual((self.state_text("last_reset_reason") or "").strip(),
                                 "supervisor:reboot_abandoned:docker_down")
                self.assertEqual(self.sysrq_text(), "")
                self.assertEqual(len(self.calls("systemctl")), 1)

    def test_stale_pending_marker_is_dropped_not_forced(self) -> None:
        self._accepted_reboot(FORCE_REBOOT_AFTER_S="180", CHECK_INTERVAL_S="30")
        self.set_uptime(1241)  # > 180 + 2 x 30: the supervisor was not running
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "")
        self.assertIn("PENDING marker stale", self.sv_log())
        self.assertEqual(self.calls("systemctl reboot -ff"), [])
        # The fault is still there, so the counted ladder issues a fresh reboot.
        self.assertEqual(len(self.calls("systemctl reboot")), 2)
        self.assertEqual(self.counter(), "2")

    # OTA probation (review finding, 2026-10-17): raising the cap alone did
    # not let u-boot's rollback fire with a carried-over counter or a held
    # file from before the update.
    def test_probation_lets_rollback_fire_despite_a_carried_counter(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0", REBOOT_MAX_CONSECUTIVE="3")
        self.set_docker_info_rc(1)
        self.put_state("consecutive_reboots", "2")  # left over from before the update
        self.set_fake("fw_upgrade_available", "1")
        self.set_fake("fw_bootlimit", "5")
        # u-boot counts each trial boot and rolls back once bootcount > bootlimit,
        # so boots 1..5 must all be allowed to reboot.
        for k in range(1, 6):
            self.set_boot(f"eeeeeeee-0000-4000-8000-{k:012d}")
            self.set_fake("fw_bootcount", str(k))
            self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 5)
        self.assertIsNone(self.state_text("held"))
        self.assertEqual(self.counter(), "7")
        self.assertIn("PROBATION overrides cap: consecutive_reboots=6 >= cap=6", self.sv_log())
        # Boot 6: u-boot rolled back and cleared upgrade_available -> normal cap.
        self.set_boot("eeeeeeee-0000-4000-8000-000000000006")
        self.set_fake("fw_upgrade_available", "0")
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 5)
        held = self.state_kv("held")
        self.assertEqual(held["cap"], "3")
        self.assertEqual(held["consecutive"], "7")

    def test_probation_overrides_a_hold_left_from_before_the_update(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0")
        self.set_docker_info_rc(1)
        self.put_state("held", "reason=docker_down\nconsecutive=3\ncap=3")
        self.put_state("consecutive_reboots", "3")
        self.set_fake("fw_upgrade_available", "1")
        self.set_fake("fw_bootcount", "1")
        self.sysrq.write_text("", encoding="utf-8")
        self.run_sv("once")
        self.assertEqual(self.calls("systemctl reboot"),
                         ["systemctl reboot [last_action=reboot consecutive=4]"])
        self.assertIn("PROBATION overrides HELD", self.sv_log())
        self.assertIsNotNone(self.state_text("held"), "the hold still needs clear-hold")
        pr = self.state_kv("probation_reboot")
        self.assertEqual((pr["boot_id"], pr["bootcount"]), (BOOT_A, "1"))
        # The forced path of an overriding reboot is not blocked by that hold.
        self.set_uptime(1180)
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "sub")

    def test_probation_override_needs_u_boot_to_count_the_reboot(self) -> None:
        # A stale env copy (flag kept in RPMB) must not disable the guard: the
        # override stops when bootcount did not rise after our last one.
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0", REBOOT_MAX_CONSECUTIVE="1")
        self.set_docker_info_rc(1)
        self.put_state("consecutive_reboots", "2")
        self.set_fake("fw_upgrade_available", "1")
        self.set_fake("fw_bootlimit", "1")  # cap = max(1, 1 + 1) = 2
        self.set_fake("fw_bootcount", "1")
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)  # overridden
        self.set_boot(BOOT_B)  # bootcount still 1
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertIn("did not rise after the supervisor's probation reboot", self.sv_log())
        self.assertEqual(self.state_kv("held")["cap"], "2")

    def test_watchdog_ping_precedes_the_reboot_rung(self) -> None:
        # The rung (syncs, systemctl, forced path) can take ~210 s by itself;
        # it starts with a fresh WatchdogSec budget.
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0")
        self.set_docker_info_rc(1)
        self.run_sv("once", extra={"NOTIFY_SOCKET": str(self.root / "notify.sock")})
        lines = self.fake_lines()
        reboot = lines.index("systemctl reboot [last_action=reboot consecutive=1]")
        self.assertIn("systemd-notify WATCHDOG=1", lines[:reboot])
        self.assertEqual(lines[-1], "systemd-notify WATCHDOG=1")  # and after the pass

    def test_heartbeat_staleness_uses_uptime_not_wall_clock(self) -> None:
        hb = self.root / "hb"
        hb.write_text("x", encoding="utf-8")
        os.utime(hb, (1, 1))  # 1970: ancient by the wall clock
        missing = self.root / "never-written"
        self.write_conf(CHECK_DOCKER="0", HEARTBEATS=f"{hb}:60 {missing}:90")

        def stale_lines() -> list[str]:
            return [ln for ln in self.sv_log().splitlines() if "STALE" in ln]

        self.set_uptime(1000)
        self.run_sv("once")
        self.set_uptime(1060)
        self.run_sv("once")
        self.assertEqual(stale_lines(), [])
        self.set_uptime(1061)
        self.run_sv("once")
        self.assertEqual(len(stale_lines()), 1)
        self.assertIn(f"heartbeat={hb} STALE", stale_lines()[0])
        self.assertIn("unchanged for 61s of uptime", stale_lines()[0])
        # Any change of the mtime value is a beat, even a step backwards.
        os.utime(hb, (0, 0))
        self.set_uptime(1091)
        self.run_sv("once")
        self.assertEqual(len(stale_lines()), 2)  # only the missing file
        self.assertIn(f"heartbeat={missing} STALE", stale_lines()[1])
        # A far-future mtime (clock step) is fresh only until it stops changing.
        os.utime(hb, (4102444800, 4102444800))  # 2100-01-01
        self.set_uptime(1100)
        self.run_sv("once")
        before = len([ln for ln in stale_lines() if f"heartbeat={hb} " in ln])
        self.set_uptime(1161)
        self.run_sv("once")
        after = len([ln for ln in stale_lines() if f"heartbeat={hb} " in ln])
        self.assertEqual(after - before, 1)
        self.assertIn("heartbeat_stale:", self.state_kv("status")["faults"])

    def test_bad_numeric_config_falls_back_per_key(self) -> None:
        self.arm()
        self.conf.write_text(
            'FAIL_THRESHOLD="abc"\n'
            "CHECK_INTERVAL_S=-5\n"
            "CONTAINER_RESTART_MAX=08\n"
            "CHECK_DOCKER=yes\n"
            "REBOOT_GRACE_S=99999999999999999999\n"
            'CONTAINERS="web_ui bad/name"\n'
            'HEARTBEATS="relative:5 /abs:x"\n',
            encoding="utf-8",
        )
        self.set_container("web_ui", "running unhealthy")
        self.passes(2)
        self.assertEqual(self.calls("docker restart"), [])  # FAIL_THRESHOLD fell back to 3
        self.passes(1)
        self.assertEqual(self.calls("docker restart"), ["docker restart web_ui"])
        log = self.sv_log()
        self.assertIn("CONFIG FAIL_THRESHOLD='abc' invalid", log)
        self.assertIn("using default 3", log)
        self.assertIn("CONFIG CHECK_INTERVAL_S='-5' invalid", log)
        self.assertIn("CONFIG CHECK_DOCKER='yes' invalid", log)
        self.assertIn("CONFIG REBOOT_GRACE_S=", log)
        self.assertNotIn("CONTAINER_RESTART_MAX", log)  # 08 is 8, not an octal error
        self.assertIn("dropping invalid container name 'bad/name'", log)
        self.assertIn("dropping invalid entry 'relative:5'", log)
        self.assertIn("dropping invalid entry '/abs:x'", log)
        self.assertEqual(self.state_kv("status")["config"], "loaded(with-defaults)")
        self.assertTrue(self.calls("docker info"))  # CHECK_DOCKER fell back to 1

    def test_unloadable_config_falls_back_to_defaults_with_role_tractor(self) -> None:
        # The file that failed may have said ROLE=tractor: it must not turn a
        # tractor's reboot rungs on, whatever it said.
        self.arm()
        self.conf.write_text("ROLE=base\nFAIL_THRESHOLD=1\nif then fi (\n", encoding="utf-8")
        self.set_docker_info_rc(1)
        self.set_uptime(5000)
        self.passes(2)
        self.assertNotIn("BLOCKED", self.sv_log())  # default threshold 3, not 1
        self.passes(1)
        log = self.sv_log()
        self.assertIn("failed to load", log)
        self.assertIn("using ALL defaults, ROLE=tractor", log)
        self.assertIn("BLOCKED reboot gate=tractor_reboot_disabled", log)
        self.assertEqual(self.calls("systemctl"), [])
        status = self.state_kv("status")
        self.assertTrue(status["config"].startswith("unloadable"))
        self.assertEqual(status["role"], "tractor")
        # An unquoted multi-word value is the likely typo. The file says
        # ROLE=base, so role=tractor below can only come from the fallback.
        self.conf.write_text("ROLE=base\nCONTAINERS=mosquitto web_ui\n", encoding="utf-8")
        self.passes(1)
        status = self.state_kv("status")
        self.assertTrue(status["config"].startswith("unloadable"), status["config"])
        self.assertEqual(status["role"], "tractor")
        self.assertEqual(self.calls("systemctl"), [])

    def test_a_line_the_shell_cannot_run_rejects_the_whole_file(self) -> None:
        # Review finding (2026-10-17): `.` only reported the LAST command's
        # status, so these lines were skipped as "not found" commands, ROLE
        # stayed at the old "base" default and an armed tractor rebooted.
        example = _CONF_EXAMPLE.read_text(encoding="utf-8")
        self.assertIn("\nROLE=tractor\n", example)
        variants = {
            # INI-style spacing on the ROLE line, mid-file (line ~35 of ~100).
            "spaced_role": example.replace("\nROLE=tractor\n", "\nROLE = tractor\n"),
            # A UTF-8 byte-order mark from a Windows editor.
            "bom": "\ufeffROLE=tractor\nFAIL_THRESHOLD=1\nREBOOT_GRACE_S=0\n",
            # Unquoted multi-word values that are not on the last line.
            "unquoted_containers": "CONTAINERS=mosq web_ui\nROLE=base\nFAIL_THRESHOLD=1\n",
            "unquoted_heartbeats": "HEARTBEATS=/a:60 /b:60\nROLE=base\nFAIL_THRESHOLD=1\n",
        }
        for name, text in variants.items():
            with self.subTest(variant=name):
                self.fresh()
                self.conf.write_text(text, encoding="utf-8")
                self.arm()
                self.set_docker_info_rc(1)
                self.set_container("mosq", "running unhealthy")
                self.set_uptime(5000)
                self.passes(3)
                log = self.sv_log()
                self.assertIn("failed to load (rc=", log)
                # The shell's own message (bash words a path-like one differently).
                self.assertRegex(log, r"CONFIG \S+: .*(not found|No such file)")
                status = self.state_kv("status")
                self.assertTrue(status["config"].startswith("unloadable"), status["config"])
                self.assertEqual(status["role"], "tractor")
                self.assertEqual(status["reboot_blocked"], "tractor_reboot_disabled")
                self.assertEqual(self.calls("systemctl"), [])
                self.assertEqual(self.calls("docker inspect"), [])  # CONTAINERS default ""

    def test_reboots_need_an_explicit_role_base(self) -> None:
        # Review finding (2026-10-17): ROLE used to default to base, so a
        # missing file, the example copied verbatim or a key typo armed a
        # tractor's reboot rungs. Now each of these reads as a tractor.
        cases = {
            "missing_file": None,
            "example_verbatim": _CONF_EXAMPLE.read_text(encoding="utf-8"),
            "no_role_line": "FAIL_THRESHOLD=1\nREBOOT_GRACE_S=0\n",
            "lower_case_key": "role=base\nFAIL_THRESHOLD=1\nREBOOT_GRACE_S=0\n",
        }
        for name, text in cases.items():
            with self.subTest(case=name):
                self.fresh()
                if text is None:
                    self.conf.unlink(missing_ok=True)
                else:
                    self.conf.write_text(text, encoding="utf-8")
                self.arm()
                self.set_docker_info_rc(1)
                self.set_uptime(5000)
                self.passes(3)
                status = self.state_kv("status")
                self.assertEqual(status["role"], "tractor")
                self.assertEqual(status["reboot_blocked"], "tractor_reboot_disabled")
                self.assertEqual(self.calls("systemctl"), [])
                log = self.sv_log()
                if name == "missing_file":
                    self.assertIn("missing: every key takes its default, including ROLE=tractor", log)
                if name in ("no_role_line", "lower_case_key"):
                    self.assertIn("CONFIG ROLE not set", log)
                    self.assertEqual(status["fail_threshold"], "1")  # the rest still loads
                if name == "lower_case_key":
                    self.assertIn("CONFIG unknown key 'role'", log)
        # And the explicit form does reboot.
        self.fresh()
        self.write_conf(ROLE="base")
        self.passes(3)
        self.assertEqual(len(self.calls("systemctl reboot")), 1)

    def test_interval_and_sysrq_settle_have_upper_bounds(self) -> None:
        # CHECK_INTERVAL_S and SYSRQ_SETTLE_S above their bounds would let the
        # unit's WatchdogSec=300 kill the supervisor mid-sleep or mid-reboot.
        self.write_conf(CHECK_INTERVAL_S="121", SYSRQ_SETTLE_S="11")
        self.run_sv("once")
        log = self.sv_log()
        self.assertIn("CONFIG CHECK_INTERVAL_S='121' invalid (want a whole number 1..120); using default 30", log)
        self.assertIn("CONFIG SYSRQ_SETTLE_S='11' invalid (want a whole number 0..10); using default 3", log)
        self.write_conf(CHECK_INTERVAL_S="120", SYSRQ_SETTLE_S="10")
        self.state.joinpath("supervisor.log").unlink()
        self.run_sv("once")
        self.assertNotIn("CONFIG", self.sv_log())

    def test_crlf_config_loads_like_lf(self) -> None:
        text = _CONF_EXAMPLE.read_text(encoding="utf-8").replace("\r\n", "\n")
        text = text.replace('CONTAINERS=""', 'CONTAINERS="mosq"')
        text = text.replace("FAIL_THRESHOLD=3", "FAIL_THRESHOLD=2")
        self.conf.write_bytes(text.replace("\n", "\r\n").encode("utf-8"))
        self.arm()
        self.set_container("mosq", "running unhealthy")
        self.passes(1)
        self.assertEqual(self.calls("docker restart"), [])
        self.passes(1)
        self.assertEqual(self.calls("docker restart"), ["docker restart mosq"])
        self.assertNotIn("CONFIG", self.sv_log())
        status = self.state_kv("status")
        self.assertEqual(status["config"], "loaded")
        self.assertEqual(status["role"], "tractor")  # what the example ships

    def test_missing_config_uses_defaults(self) -> None:
        self.conf.unlink()
        self.run_sv("once")
        status = self.state_kv("status")
        self.assertEqual(status["config"], "missing(defaults)")
        self.assertEqual(status["role"], "tractor")  # fail-safe default
        self.assertEqual(status["fail_threshold"], "3")
        self.assertIn("CONFIG", self.sv_log())

    def test_invalid_role_is_treated_as_tractor(self) -> None:
        self.arm()
        self.write_conf(ROLE="tracter", FAIL_THRESHOLD="1", REBOOT_GRACE_S="0")
        self.set_docker_info_rc(1)
        self.run_sv("once")
        self.assertIn("CONFIG ROLE='tracter' invalid", self.sv_log())
        self.assertIn("BLOCKED reboot gate=tractor_reboot_disabled", self.sv_log())
        self.assertEqual(self.calls("systemctl"), [])

    def test_example_config_loads_clean_and_restarts_nothing(self) -> None:
        self.conf.write_text(_CONF_EXAMPLE.read_text(encoding="utf-8"), encoding="utf-8")
        self.arm()
        self.set_container("design-controller-mosquitto-1", "running unhealthy")
        self.passes(4)
        self.assertNotIn("CONFIG", self.sv_log())
        status = self.state_kv("status")
        self.assertEqual(status["config"], "loaded")
        self.assertEqual(status["role"], "tractor")
        self.assertEqual(self.calls("docker inspect"), [])
        self.assertEqual(self.calls("docker restart"), [])

    def test_duplicate_allowlist_entries_count_once(self) -> None:
        # Listed twice, a container was inspected twice per pass and restarted
        # after ceil(FAIL_THRESHOLD / 2) passes.
        self.arm()
        hb = self.root / "hb"
        self.write_conf(CONTAINERS="mosq mosq", HEARTBEATS=f"{hb}:60 {hb}:90")
        self.set_container("mosq", "running unhealthy")
        self.passes(2)
        self.assertEqual(self.calls("docker restart"), [])
        self.passes(1)
        self.assertEqual(self.calls("docker restart"), ["docker restart mosq"])
        self.assertEqual(len(self.calls("docker inspect")), 3)
        log = self.sv_log()
        self.assertIn("CONFIG CONTAINERS: ignoring duplicate 'mosq'", log)
        self.assertIn(f"CONFIG HEARTBEATS: ignoring duplicate path '{hb}'", log)

    def test_hex_only_container_names_are_refused(self) -> None:
        # Docker resolves an unknown name as a container ID prefix, so
        # `docker restart abc123` could restart some other container.
        self.arm()
        self.write_conf(CONTAINERS="abc123 DEADBEEF mosq", FAIL_THRESHOLD="1")
        self.set_container("abc123", "running unhealthy")
        self.set_container("DEADBEEF", "running unhealthy")
        self.set_container("mosq", "running healthy")
        self.run_sv("once")
        inspects = self.calls("docker inspect")
        self.assertEqual(len(inspects), 1)
        self.assertTrue(inspects[0].endswith(" mosq"), inspects[0])
        self.assertIn("dropping 'abc123': hex digits only", self.sv_log())
        self.assertIn("dropping 'DEADBEEF': hex digits only", self.sv_log())
        self.assertEqual(self.calls("docker restart"), [])

    def test_container_resolving_to_another_name_is_never_restarted(self) -> None:
        self.arm()
        self.write_conf(CONTAINERS="mosq", FAIL_THRESHOLD="1", CONTAINER_RESTART_MAX="2")
        self.set_container("mosq", "running unhealthy", resolves="/design-controller-web_ui-1")
        self.passes(3)
        self.assertEqual(self.calls("docker restart"), [])
        log = self.sv_log()
        self.assertEqual(log.count("REFUSED restart container=mosq"), 2)
        self.assertIn("resolves_to=/design-controller-web_ui-1", log)
        # Counted like a missing container, so it escalates the same way.
        self.assertIn("restarts used up (2/2 this boot)", log)
        self.assertEqual(len(self.calls("systemctl reboot")), 1)

    def test_unusable_state_dir_writes_nothing_to_stderr(self) -> None:
        # `printf > "$f" 2>/dev/null` reports a failed open on the original
        # stderr (redirections run left to right); every pass used to spray
        # "cannot create" lines into the journal.
        blocker = self.root / "not-a-dir"
        blocker.write_text("x", encoding="utf-8")
        self.arm()
        self.write_conf(CONTAINERS="mosq", HEARTBEATS=f"{self.root / 'hb'}:60", FAIL_THRESHOLD="1")
        self.set_container("mosq", "running unhealthy")
        # run_sv fails the test on any stderr output.
        cp = self.run_sv("once", extra={"LIFETRAC_SUPERVISOR_STATE": str(blocker / "state")})
        self.assertIn("ERROR state dir", cp.stdout)
        self.assertIn("WOULD restart container=mosq", cp.stdout)  # unusable state = disarmed
        self.assertEqual(self.calls("docker restart"), [])

    # ---------------------------------------- tractor idle gate (D-PWR1)
    # Dorkmo, 2026-10-18: a tractor reboots itself only when idle. The H747
    # reports idle through its link service and holds motion off for the
    # reboot; the supervisor never resets a tractor without both.
    def _tractor(self, **conf: str) -> None:
        self.arm()
        settings = {"ROLE": "tractor", "TRACTOR_REBOOT_ENABLED": "1",
                    "FAIL_THRESHOLD": "1", "REBOOT_GRACE_S": "0"}
        settings.update(conf)
        self.write_conf(**settings)
        self.set_docker_info_rc(1)
        self.sysrq.write_text("", encoding="utf-8")
        self.set_uptime(1000)

    def assert_nothing_rebooted(self) -> None:
        self.assertEqual(self.calls("systemctl"), [])
        self.assertEqual(self.sysrq_text(), "")
        self.assertIsNone(self.state_text("last_action"))
        self.assertIsNone(self.state_text("consecutive_reboots"))
        self.assertIsNone(self.request())

    def test_tractor_reboot_needs_a_live_idle_report(self) -> None:
        cases = {
            "missing": (None, "missing:"),
            "frozen_idle": ("idle 7", "not_live:seq_unchanged_for_10_polls"),
            "frozen_lock": ("locked 7 x", "not_live:"),
            "bad_seq": ("idle 7!", "not_live:"),
            "busy": ("busy", "state=busy"),
            "unknown": ("parked", "state=parked"),
        }
        for case, (line, why) in cases.items():
            with self.subTest(case=case):
                self.fresh()
                self._tractor()
                if self.idle_file.exists():
                    self.idle_file.unlink()
                link = None
                if line == "busy":
                    link = self.start_link("busy")
                elif line == "parked":
                    link = self.start_link("parked")
                elif line is not None:
                    self.write_idle(line)
                self.run_sv("once")
                if link is not None:
                    link.stop()
                self.assert_nothing_rebooted()
                self.assertIn("BLOCKED reboot gate=tractor_not_idle", self.sv_log())
                status = self.state_kv("status")
                self.assertEqual(status["reboot_blocked"], "tractor_not_idle")
                self.assertIn("reboot_blocked:tractor_not_idle", status["decision"])
                self.assertIn(why, status["tractor_idle"])
                # The node streak is kept: the next pass asks again.
                self.assertEqual(status["node_fault_streak"], "1")

    def test_tractor_reboots_once_the_h747_holds_motion_off(self) -> None:
        self._tractor()
        link = self.start_link("busy")
        self.run_sv("once")
        self.assert_nothing_rebooted()
        link.mode = "grant"
        self.set_uptime(1030)
        self.run_sv("once")
        self.assertEqual(self.calls("systemctl reboot"),
                         ["systemctl reboot [last_action=reboot consecutive=1]"])
        # The request was on disk, and the H747 held motion off for it, when
        # systemd took the reboot; it stays on disk while the reboot is
        # pending (tmpfs: gone with the reboot).
        want = "aaaaaaaa-1030"
        self.assertEqual(link.requests, [want])
        self.assertEqual((self.fake / "request.at_reboot").read_text(encoding="utf-8").strip(), want)
        self.assertEqual(self.request(), want)
        log = self.sv_log()
        self.assertLess(log.index("LOCK requested id=" + want), log.index("LOCK held id=" + want))
        self.assertLess(log.index("LOCK held id=" + want), log.index("ACTION reboot reason=docker_down"))
        self.assertEqual(self.state_kv("status")["tractor_idle"], "locked=" + want)

    def test_tractor_reboot_is_blocked_when_the_hold_is_not_confirmed(self) -> None:
        for mode in ("idle", "stale_lock"):
            with self.subTest(mode=mode):
                self.fresh()
                self._tractor()
                link = self.start_link(mode)
                self.run_sv("once")
                link.stop()
                # Asked once, then withdrawn: the H747 must not be left
                # holding motion off for a reboot that is not coming.
                self.assertEqual(link.requests, ["aaaaaaaa-1000"])
                self.assert_nothing_rebooted()
                log = self.sv_log()
                self.assertIn("BLOCKED reboot gate=tractor_lock", log)
                self.assertIn("LOCK withdrawn (not granted: no_lock:state=", log)
                status = self.state_kv("status")
                self.assertEqual(status["reboot_blocked"], "tractor_lock")
                self.assertIn("reboot_blocked:tractor_lock", status["decision"])
                if mode == "stale_lock":
                    # A hold for some other request reads as idle, never as ours.
                    self.assertIn("locked:" + _FakeLink.STALE_ID, status["tractor_idle"])

    def test_tractor_disarmed_probes_idle_but_never_asks_for_the_hold(self) -> None:
        self._tractor()
        self.armed.unlink()
        link = self.start_link("grant")
        self.run_sv("once")
        self.assertEqual(link.requests, [])
        self.assertIsNone(self.request())
        self.assertIn("WOULD reboot reason=docker_down", self.sv_log())
        self.assertIn("the H747 reports idle", self.sv_log())
        link.mode = "busy"
        self.set_uptime(1030)
        self.run_sv("once")
        self.assertIn("BLOCKED reboot gate=tractor_not_idle", self.sv_log())
        self.assertEqual(self.calls("systemctl"), [])

    def test_tractor_probation_never_overrides_the_idle_gate(self) -> None:
        self._tractor()
        self.put_state("held", "reason=old\nconsecutive=3\ncap=3")
        self.set_fake("fw_upgrade_available", "1")
        self.set_fake("fw_bootlimit", "5")
        self.set_fake("fw_bootcount", "1")
        self.start_link("busy")
        self.run_sv("once")
        self.assertIn("PROBATION overrides HELD", self.sv_log())
        self.assertIn("BLOCKED reboot gate=tractor_not_idle", self.sv_log())
        self.assertEqual(self.calls("systemctl"), [])

    def test_tractor_forced_path_needs_the_hold(self) -> None:
        # systemd took the reboot at up=1000; at 1180 the supervisor is still
        # alive with the fault present, so the forced path is due.
        cases = {
            # link still up and holding: force.
            "held": ("grant", True),
            # link stopped by the shutdown (file left, <seq> frozen): the H747
            # keeps the hold until it resets, so force.
            "link_stopped": ("stop", True),
            # link live but the H747 no longer reports our hold: never force.
            "hold_lost": ("busy", False),
            "idle_no_hold": ("idle", False),
            # request file gone: nothing says the H747 is holding.
            "request_gone": ("rm_request", False),
        }
        for case, (what, forced) in cases.items():
            with self.subTest(case=case):
                self.fresh()
                self.sysrq.write_text("", encoding="utf-8")
                if hasattr(self, "link"):
                    self.link.stop()
                    del self.link
                self._accepted_reboot(ROLE="tractor", TRACTOR_REBOOT_ENABLED="1")
                req = self.request()
                self.assertEqual(req, "aaaaaaaa-1000")
                if what == "stop":
                    self.link.stop()
                elif what == "rm_request":
                    self.req_file.unlink()
                else:
                    self.link.mode = what
                self.set_uptime(1180)
                self.run_sv("once")
                log = self.sv_log()
                if forced:
                    self.assertEqual(self.sysrq_text(), "sub")
                    self.assertEqual(self.calls("systemctl reboot -ff"),
                                     ["systemctl reboot -ff [last_action=force_reboot consecutive=1]"])
                    self.assertEqual((self.fake / "request.at_reboot_ff").read_text(encoding="utf-8").strip(), req)
                else:
                    self.assertEqual(self.sysrq_text(), "")
                    self.assertEqual(self.calls("systemctl reboot -ff"), [])
                    self.assertIn("BLOCKED forced reboot gate=tractor_lock", log)
                    la = self.state_kv("last_action")
                    self.assertEqual(la["action"], "reboot_abandoned")
                    self.assertTrue(la["detail"].startswith("force_blocked:tractor_lock:alive_180s"), la["detail"])
                    status = self.state_kv("status")
                    self.assertEqual(status["reboot_blocked"], "tractor_lock")
                    self.assertIn("force_blocked:tractor_lock", status["decision"])
                # Forced or not, this boot's attempt is over and the forced
                # path that returned (fakes) released the hold.
                self.assertIsNone(self.request())
                self.assertFalse(self.pending_marker().exists())

    def test_tractor_systemctl_failure_forces_while_the_hold_is_live(self) -> None:
        self._tractor()
        self.set_fake("systemctl_reboot_rc", "1")
        self.start_link("grant")
        self.run_sv("once")
        self.assertEqual(self.sysrq_text(), "sub")
        self.assertEqual((self.fake / "request.at_reboot_ff").read_text(encoding="utf-8").strip(),
                         "aaaaaaaa-1000")
        self.assertIn("LOCK still held id=aaaaaaaa-1000", self.sv_log())
        # The fakes return, so the forced path came back: hold released.
        self.assertIsNone(self.request())

    def test_tractor_abandoned_reboot_withdraws_the_request(self) -> None:
        self._accepted_reboot(ROLE="tractor", TRACTOR_REBOOT_ENABLED="1")
        self.assertEqual(self.request(), "aaaaaaaa-1000")
        self.set_uptime(1100)
        self.run_sv("once")  # inside the window: still pending, still held
        self.assertEqual(self.request(), "aaaaaaaa-1000")
        self.set_docker_info_rc(0)
        self.set_uptime(1180)
        self.run_sv("once")
        self.assertIn("no node fault now: not forcing", self.sv_log())
        self.assertIsNone(self.request())
        self.assertEqual(self.sysrq_text(), "")

    def test_tractor_refused_reboot_withdraws_the_request(self) -> None:
        # The counter cannot be persisted: no reboot, and the H747 must not be
        # left holding motion off.
        self._tractor()
        self.state.mkdir(parents=True)
        (self.state / "consecutive_reboots.tmp").mkdir()
        self.start_link("grant")
        self.run_sv("once")
        self.assertIn("cannot persist consecutive_reboots", self.sv_log())
        self.assertIn("LOCK withdrawn (reboot refused)", self.sv_log())
        self.assertIsNone(self.request())
        self.assertEqual(self.calls("systemctl"), [])

    def test_request_left_without_a_pending_reboot_is_withdrawn_at_start(self) -> None:
        for role in ("tractor", "base"):
            with self.subTest(role=role):
                self.fresh()
                self.write_conf(ROLE=role)
                self.req_file.parent.mkdir(parents=True, exist_ok=True)
                self.req_file.write_text("aaaaaaaa-900\n", encoding="utf-8")
                self.run_sv("once")
                self.assertIsNone(self.request())
                self.assertIn("LOCK withdrawn (left from an earlier run with no pending reboot)",
                              self.sv_log())

    def test_base_never_reads_the_idle_file_or_asks_for_a_hold(self) -> None:
        self.arm()
        self.write_conf(FAIL_THRESHOLD="1", REBOOT_GRACE_S="0")
        self.set_docker_info_rc(1)
        self.write_idle("busy 1")
        self.run_sv("once")
        self.assertEqual(len(self.calls("systemctl reboot")), 1)
        self.assertIsNone(self.request())
        self.assertEqual(self.state_kv("status")["tractor_idle"], "not_checked")
        self.assertNotIn("LOCK", self.sv_log())

    def test_idle_wait_bounds(self) -> None:
        for value, want in (("1", "10"), ("16", "10"), ("x", "10"), ("2", "2"), ("15", "15")):
            with self.subTest(value=value):
                self.fresh()
                self._tractor(TRACTOR_IDLE_WAIT_S=value)
                self.write_idle("idle 1")
                self.run_sv("once")
                self.assertIn(f"not_live:seq_unchanged_for_{want}_polls",
                              self.state_kv("status")["tractor_idle"])
                if want != value:
                    self.assertIn(f"CONFIG TRACTOR_IDLE_WAIT_S='{value}' invalid", self.sv_log())

    def test_watchdog_notify_only_with_notify_socket(self) -> None:
        self.run_sv("once")
        self.assertEqual(self.calls("systemd-notify"), [])
        self.run_sv("once", extra={"NOTIFY_SOCKET": str(self.root / "notify.sock")})
        self.assertEqual(self.calls("systemd-notify"), ["systemd-notify WATCHDOG=1"])

    def test_log_rotates_past_256_kib(self) -> None:
        self.state.mkdir(parents=True)
        (self.state / "supervisor.log").write_text("x" * 300_000, encoding="utf-8")
        self.set_docker_info_rc(1)
        self.run_sv("once")
        self.assertEqual((self.state / "supervisor.log.1").stat().st_size, 300_000)
        self.assertLess((self.state / "supervisor.log").stat().st_size, 4096)
        self.assertIn("CHECK docker FAIL", self.sv_log())

    def test_status_command_reads_only(self) -> None:
        cp = self.run_sv("status")
        self.assertIn("absent: disarmed", cp.stdout)
        self.assertFalse(self.state.exists(), "status must not create state")
        self.run_sv("once")
        self.arm()
        cp = self.run_sv("status")
        self.assertIn("present: ARMED", cp.stdout)
        self.assertIn("--- status", cp.stdout)
        self.assertIn("role=base", cp.stdout)

    def test_unknown_command_is_rejected(self) -> None:
        cp = subprocess.run([self.shell, str(_SCRIPT), "reboot-now"], env=self.env(),
                            capture_output=True, text=True, timeout=30)
        self.assertEqual(cp.returncode, 2)
        self.assertEqual(self.fake_lines(), [])


@unittest.skipUnless(_HAVE_POSIX and shutil.which("bash"), "needs bash")
class X8SUP_C_SupervisorBash(X8SUP_B_Supervisor):
    SHELL = "bash"


if __name__ == "__main__":
    unittest.main()
