"""PWR-2 / L2 -- systemd owns the X8 hardware watchdog (SIL).

X8_HEALTH_AND_RECOVERY/REBOOT_AND_POWER_CYCLE.md layer L2: u-boot arms the
i.MX WDOG1 at 60 s (WDOG_B -> BD71847 PMIC); ``firmware/x8_recovery`` hands
it to systemd (RuntimeWatchdogSec=30s, RebootWatchdogSec=2min) and bounds a
hung reboot (reboot.target JobTimeoutSec=3min). /dev/watchdog0 is
single-open, so the flash pipeline's ``wdt_pet.sh`` must notice the owner and
rely on it instead of reporting "failed to start".

WDO_A_DropIns
    Both drop-ins parse INI-style with exactly the expected section, keys and
    values; every timeout stays within imx2_wdt's 128 s hardware maximum;
    LF endings; the bench and tractor gates are still spelled out.

WDO_B_WdtPetSystemdOwned
    ``wdt_pet.sh start`` with a fake ``systemctl`` reporting an armed
    RuntimeWatchdogUSec ("30s", "2min", "infinity") exits 0, says it relies
    on systemd, shows the fuser holder, writes no pidfile (and clears a stale
    one), never starts the python petter, and leaves one "RELY systemd" line
    in the pet log (run_flash_bench.sh tails that log after every flash).

WDO_C_WdtPetPetter
    With systemd reporting "0" (or not answering) the petter runs: against a
    regular file standing in for the device it starts, pets, and ``stop``
    leaves 'V' as the last byte; a nonexistent device is a start failure
    (exit 2, no pidfile); an open() that gets EBUSY (an exclusive pty, the
    only EBUSY a test can make without root or a real watchdog) is reported
    as owned elsewhere (exit 0, no pidfile); a petter blocked in open()
    (a FIFO with no reader) is not reported as started and still closes
    with 'V' once it gets the device; a non-numeric or zero
    WDT_PET_START_POLLS falls back to 15 polls instead of hanging or giving
    up on a healthy petter.

WDO_D_Installer
    ``install_watchdog.sh`` install/remove/check against a temp
    LIFETRAC_ETC_SYSTEMD with fake systemctl/fuser: byte-identical copies,
    idempotent, never runs daemon-reexec/reload/reboot (it prints
    daemon-reload: systemd 250 re-reads system.conf.d on every reload), check
    reports installed/partial/differs/absent, tells "PID 1 has not re-read
    it" apart from "systemctl gave no answer" and flags an armed watchdog
    with no drop-in, and says "present-unverified" (not "differs") when
    only the script was pushed, finding the repo copies through a symlinked
    invocation; --role is required, tractor is refused without
    --i-accept-tractor-prereqs (naming PWR-8, PWR-9 and PWR-11), bench is
    refused on a board without the bench sudoers marker and needs
    --bench-opt-in where it has one, the marker makes base and tractor need
    --bench-opt-in too, base prints the PWR-1 OTA/shutdown-timing caveat,
    install tells the operator to stop a live wdt_pet petter before any
    reload, and a refusal writes nothing.

WDO_E_ReadOnlyTripwires
    ``capture_board_state.sh`` (outside comments) has no i2c tool, no
    redirect into /proc/sysrq-trigger, /proc/sys or /dev/watchdog*, and no
    fw_setenv; its header still lists those as never-done. Every script this
    piece touched passes ``bash -n`` / ``dash -n``.

Hermetic: fakes live in a temp dir prepended to PATH, every path the scripts
use is overridden by env var, nothing needs root, and nothing opens a real
watchdog, writes /proc or /etc. The EBUSY case needs a non-root uid (root
bypasses TIOCEXCL) and is skipped when run as root.
"""

from __future__ import annotations

import configparser
import os
import re
import shutil
import signal
import stat
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_DC = _BS.parent                                     # DESIGN-CONTROLLER/
_FW = _DC / "firmware"
_REC = _FW / "x8_recovery"
_HELPER = _FW / "x8_lora_bootloader_helper"

WDT_PET = _HELPER / "wdt_pet.sh"
FLASH_PIPELINE = _HELPER / "full_flash_pipeline.sh"
CAPTURE = _HELPER / "bench_tools" / "capture_board_state.sh"
INSTALLER = _REC / "install_watchdog.sh"
WD_DROPIN = _REC / "systemd" / "system.conf.d" / "90-lifetrac-watchdog.conf"
RT_DROPIN = _REC / "systemd" / "reboot.target.d" / "90-lifetrac-timeout.conf"

BASH = shutil.which("bash")
DASH = shutil.which("dash")
PY3 = shutil.which("python3")

IMX2_WDT_MAX_S = 128   # drivers/watchdog/imx2_wdt.c IMX2_WDT_MAX_TIME

# fake systemctl: logs every call, answers the queries the scripts make.
# FAKE_RWD unset -> "0"; FAKE_RWD=__fail__ -> no output, exit 1 ("not booted
# with systemd").
_FAKE_SYSTEMCTL = """#!/bin/sh
echo "$*" >> "$FAKE_CALLS"
[ "${FAKE_RWD-0}" = __fail__ ] && { echo "System has not been booted with systemd" >&2; exit 1; }
case "$*" in
  *reboot.target*) printf 'JobTimeoutUSec=3min\\nJobTimeoutAction=reboot-force\\n' ;;
  *--value*) echo "${FAKE_RWD-0}" ;;
  *) printf 'RuntimeWatchdogUSec=%s\\nRebootWatchdogUSec=%s\\n' "${FAKE_RWD-0}" 10min ;;
esac
"""
# fake fuser: psmisc prints pids on stdout and the "dev:" label on stderr
_FAKE_FUSER = """#!/bin/sh
echo "fuser $*" >> "$FAKE_CALLS"
echo "$1:" >&2
echo 1
"""
_FAKE_SYSTEMD_ANALYZE = """#!/bin/sh
echo "systemd-analyze $*" >> "$FAKE_CALLS"
printf '# /etc/systemd/system.conf\\n#RuntimeWatchdogSec=off\\n'
"""


def _systemd_seconds(value: str) -> float:
    """The subset of systemd.time(7) the drop-ins use: '30s', '2min', '3min'."""
    m = re.fullmatch(r"(\d+)(ms|s|min|h)", value.strip())
    if not m:
        raise ValueError("unexpected systemd timespan %r" % value)
    return int(m.group(1)) * {"ms": 0.001, "s": 1, "min": 60, "h": 3600}[m.group(2)]


def _parse_dropin(path: Path) -> configparser.ConfigParser:
    cp = configparser.ConfigParser(
        interpolation=None, comment_prefixes=("#", ";"), strict=True)
    cp.optionxform = str  # systemd keys are case-sensitive
    cp.read_string(path.read_text(encoding="ascii"))
    return cp


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    # a zombie answers kill(0) too; treat it as gone
    try:
        with open("/proc/%d/stat" % pid) as f:
            return f.read().rsplit(")", 1)[1].split()[0] != "Z"
    except OSError:
        return True


class _ScriptHarness(unittest.TestCase):
    """Temp dir with fake bin/, a pet dir and a log; env with nothing inherited
    from a real WDT_*/LIFETRAC_* setting."""

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="wdo_sil_"))
        self.bin = self.tmp / "bin"
        self.bin.mkdir()
        for name, body in (("systemctl", _FAKE_SYSTEMCTL), ("fuser", _FAKE_FUSER),
                           ("systemd-analyze", _FAKE_SYSTEMD_ANALYZE)):
            p = self.bin / name
            p.write_text(body)
            p.chmod(0o755)
        self.pet_dir = self.tmp / "p0c"
        self.log = self.tmp / "wdt_pet.log"
        self.calls = self.tmp / "calls.log"
        self.etc = self.tmp / "etc_systemd"
        # absent unless a test creates it; never the real /etc/sudoers.d
        self.bench_marker = self.tmp / "sudoers.d" / "99-lifetrac-bench-nopasswd"
        env = {k: v for k, v in os.environ.items()
               if not k.startswith(("WDT_", "LIFETRAC_", "FAKE_"))}
        env.update(PATH="%s:%s" % (self.bin, os.environ.get("PATH", "/usr/bin:/bin")),
                   WDT_PET_DIR=str(self.pet_dir), WDT_PET_LOG=str(self.log),
                   WDT_DEV=str(self.tmp / "watchdog0"), WDT_PET_INTERVAL_S="0.3",
                   # generous for a loaded CI runner; the wait ends at the
                   # first OPEN/BUSY line, so a healthy start costs ~0.1 s
                   WDT_PET_START_POLLS="100",
                   LIFETRAC_ETC_SYSTEMD=str(self.etc),
                   LIFETRAC_BENCH_MARKER=str(self.bench_marker),
                   FAKE_CALLS=str(self.calls))
        self.env = env
        self.fifos: list[Path] = []

    def tearDown(self) -> None:
        # never leave a petter behind: release any FIFO it may block on, then
        # TERM (it hands the fake device back with 'V'), then KILL as a last
        # resort -- these are temp files, not a real watchdog
        for f in self.fifos:
            try:
                fd = os.open(f, os.O_RDONLY | os.O_NONBLOCK)
                time.sleep(0.3)
                os.close(fd)
            except OSError:
                pass
        for pid in self._petter_pids():
            if not self._is_our_petter(pid):   # dead, or a reused pid: leave it alone
                continue
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.kill(pid, sig)
                except OSError:
                    break
                deadline = time.monotonic() + 3
                while _pid_alive(pid) and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not _pid_alive(pid):
                    break
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _petter_pids(self) -> set[int]:
        pids = set()
        pidfile = self.pet_dir / "wdt_pet.pid"
        if pidfile.exists():
            txt = pidfile.read_text().strip()
            if txt.isdigit():
                pids.add(int(txt))
        if self.log.exists():
            pids.update(int(m) for m in re.findall(r"pid=(\d+)", self.log.read_text()))
        return pids

    def _is_our_petter(self, pid: int) -> bool:
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as f:
                return str(self.log).encode() in f.read()
        except OSError:
            return False

    def wdt(self, *args: str, **env: str) -> subprocess.CompletedProcess:
        e = dict(self.env, **env)
        return subprocess.run([BASH, str(WDT_PET), *args], env=e,
                              capture_output=True, text=True, timeout=60)

    def installer(self, *args: str, **env: str) -> subprocess.CompletedProcess:
        e = dict(self.env, **env)
        return subprocess.run([DASH or "sh", str(INSTALLER), *args], env=e,
                              capture_output=True, text=True, timeout=60)

    def calls_text(self) -> str:
        return self.calls.read_text() if self.calls.exists() else ""


# --------------------------------------------------------------------------
class WDO_A_DropIns(unittest.TestCase):

    def test_watchdog_dropin_keys(self) -> None:
        cp = _parse_dropin(WD_DROPIN)
        self.assertEqual(cp.sections(), ["Manager"])
        self.assertEqual(dict(cp["Manager"]),
                         {"RuntimeWatchdogSec": "30s", "RebootWatchdogSec": "2min"})

    def test_reboot_target_dropin_keys(self) -> None:
        cp = _parse_dropin(RT_DROPIN)
        self.assertEqual(cp.sections(), ["Unit"])
        self.assertEqual(dict(cp["Unit"]),
                         {"JobTimeoutSec": "3min", "JobTimeoutAction": "reboot-force"})

    def test_timeouts_fit_the_hardware_and_each_other(self) -> None:
        wd = _parse_dropin(WD_DROPIN)["Manager"]
        runtime = _systemd_seconds(wd["RuntimeWatchdogSec"])
        reboot = _systemd_seconds(wd["RebootWatchdogSec"])
        job = _systemd_seconds(_parse_dropin(RT_DROPIN)["Unit"]["JobTimeoutSec"])
        # within imx2_wdt's maximum, so the hardware counts the value asked for
        self.assertLessEqual(runtime, IMX2_WDT_MAX_S)
        self.assertLessEqual(reboot, IMX2_WDT_MAX_S)
        # tighter than u-boot's 60 s, and far below upstream's 30 min job timeout
        self.assertLess(runtime, 60)
        self.assertLess(job, 30 * 60)

    def test_dropins_are_lf_ascii_and_keep_their_gates(self) -> None:
        for p in (WD_DROPIN, RT_DROPIN, INSTALLER):
            raw = p.read_bytes()
            self.assertNotIn(b"\r", raw, p.name)
            self.assertTrue(raw.endswith(b"\n"), p.name)
            raw.decode("ascii")
        text = WD_DROPIN.read_text()
        for needle in ("BENCH", "TRACTOR", "/dev/ttymxc3", "128 s", "systemd-shutdown",
                       "PWR-8", "PWR-9", "PWR-11", "daemon-reload"):
            self.assertIn(needle, text)
        # systemd 250 re-reads system.conf.d on every daemon-reload; the old
        # "only when PID 1 starts" claim must not come back
        self.assertNotIn("only when it starts", text)
        self.assertNotIn("only when it starts", INSTALLER.read_text())
        # the reboot bound names its cost to a staged OTA
        self.assertIn("ostree-finalize-staged", RT_DROPIN.read_text())
        self.assertTrue(WD_DROPIN.name.startswith("90-"))  # sorts after a vendor 10-*.conf


# --------------------------------------------------------------------------
@unittest.skipUnless(BASH and PY3, "bash and python3 required")
class WDO_B_WdtPetSystemdOwned(_ScriptHarness):

    def test_armed_values_mean_rely_on_systemd(self) -> None:
        for rwd in ("30s", "2min", "infinity"):
            with self.subTest(RuntimeWatchdogUSec=rwd):
                r = self.wdt("start", FAKE_RWD=rwd)
                self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
                self.assertIn("systemd owns %s (RuntimeWatchdogUSec=%s)" % (self.env["WDT_DEV"], rwd),
                              r.stdout)
                self.assertIn("relying on it, no petter started", r.stdout)
                self.assertIn("fuser %s: 1(" % self.env["WDT_DEV"], r.stdout)
                self.assertNotIn("failed to start", r.stdout)
                self.assertFalse((self.pet_dir / "wdt_pet.pid").exists())
                # the log says why there are no pets, and exists for a tail
                self.assertRegex(self.log.read_text().splitlines()[-1],
                                 r"^[0-9.]* RELY systemd RuntimeWatchdogUSec=%s, no petter started$"
                                 % re.escape(rwd))
        log = self.log.read_text()
        self.assertEqual(log.count(" RELY systemd "), 3)
        for petter_line in ("OPEN", "PET ", "BUSY", "CLOSE-WITH-MAGIC"):
            self.assertNotIn(petter_line, log, "the python petter must not run")
        self.assertIn("show -p RuntimeWatchdogUSec --value", self.calls_text())

    def test_stale_pidfile_is_cleared(self) -> None:
        self.pet_dir.mkdir()
        (self.pet_dir / "wdt_pet.pid").write_text("999999999\n")  # above pid_max: dead
        r = self.wdt("start", FAKE_RWD="30s")
        self.assertEqual(r.returncode, 0)
        self.assertFalse((self.pet_dir / "wdt_pet.pid").exists())
        r = self.wdt("stop")
        self.assertEqual(r.stdout.strip(), "wdt_pet not running")



# --------------------------------------------------------------------------
@unittest.skipUnless(BASH and PY3, "bash and python3 required")
class WDO_C_WdtPetPetter(_ScriptHarness):

    def test_regular_file_device_start_status_stop(self) -> None:
        dev = Path(self.env["WDT_DEV"])
        dev.write_bytes(b"")
        for rwd in ("0", "", "__fail__"):   # off, unset, systemctl not answering
            with self.subTest(RuntimeWatchdogUSec=rwd):
                dev.write_bytes(b"")
                r = self.wdt("start", FAKE_RWD=rwd)
                self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
                m = re.search(r"wdt_pet started pid=(\d+) ", r.stdout)
                self.assertIsNotNone(m, r.stdout)
                pid = int(m.group(1))
                self.assertEqual((self.pet_dir / "wdt_pet.pid").read_text().strip(), str(pid))
                self.assertIn("OPEN pid=%d " % pid, self.log.read_text())

                r = self.wdt("start", FAKE_RWD=rwd)
                self.assertIn("already running pid=%d" % pid, r.stdout)
                self.assertIn("wdt_pet running pid=%d" % pid, self.wdt("status").stdout)

                time.sleep(0.5)            # at least one more pet
                n_close = self.log.read_text().count("CLOSE-WITH-MAGIC")
                r = self.wdt("stop")
                self.assertIn("wdt_pet stopped pid=%d" % pid, r.stdout)
                # stop waits up to 3 s; give a slow runner longer to finish
                deadline = time.monotonic() + 10
                while (self.log.read_text().count("CLOSE-WITH-MAGIC") == n_close
                       and time.monotonic() < deadline):
                    time.sleep(0.05)
                self.assertEqual(self.log.read_text().count("CLOSE-WITH-MAGIC"), n_close + 1)
                data = dev.read_bytes()
                self.assertTrue(data.startswith(b"\0"), data)
                self.assertTrue(data.endswith(b"V"), data)      # magic close last
                self.assertEqual(data.count(b"V"), 1)
                self.assertFalse((self.pet_dir / "wdt_pet.pid").exists())
                self.assertEqual(self.wdt("stop").stdout.strip(), "wdt_pet not running")
                self.assertEqual(self.wdt("status").stdout.strip(), "wdt_pet NOT running")

    def test_bad_start_polls_fall_back_to_15(self) -> None:
        # zero would kill a healthy petter at the first poll: it must start
        dev = Path(self.env["WDT_DEV"])
        dev.write_bytes(b"")
        r = self.wdt("start", FAKE_RWD="0", WDT_PET_START_POLLS="0")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("wdt_pet started pid=", r.stdout)
        self.assertIn("wdt_pet stopped pid=", self.wdt("stop").stdout)
        # a non-number used to make the wait endless with a silent petter;
        # now it gives up after ~3 s (15 polls)
        fifo = self.tmp / "fifo_watchdog"
        os.mkfifo(fifo)
        self.fifos.append(fifo)
        t0 = time.monotonic()
        r = self.wdt("start", FAKE_RWD="0", WDT_DEV=str(fifo), WDT_PET_START_POLLS="abc")
        self.assertLess(time.monotonic() - t0, 30)
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn("never logged OPEN", r.stdout)
        self.assertNotIn("integer expression expected", r.stdout + r.stderr)

    def test_nonexistent_device_is_a_start_failure(self) -> None:
        r = self.wdt("start", WDT_DEV=str(self.tmp / "no_such_watchdog"))
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn("wdt_pet failed to start", r.stdout)
        self.assertNotIn("relying on it", r.stdout)
        self.assertFalse((self.pet_dir / "wdt_pet.pid").exists())
        self.assertIn("OPEN-FAILED", self.log.read_text())
        self.assertIn("errno=2", self.log.read_text())

    def test_ebusy_open_is_owned_elsewhere(self) -> None:
        if os.geteuid() == 0:
            self.skipTest("root bypasses TIOCEXCL, so the pty cannot return EBUSY; "
                          "this runs in CI (non-root)")
        try:
            import fcntl
            import termios
            master, slave = os.openpty()
        except (ImportError, OSError) as exc:
            self.skipTest("no pty to make an EBUSY device: %s" % exc)
        try:
            # an exclusive tty refuses every further open() by a non-root uid
            # with EBUSY -- the same errno a second open of /dev/watchdog0 gets
            fcntl.ioctl(slave, termios.TIOCEXCL)
            busy_dev = os.ttyname(slave)
            r = self.wdt("start", FAKE_RWD="0", WDT_DEV=busy_dev)
        finally:
            os.close(slave)
            os.close(master)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("%s owned elsewhere" % busy_dev, r.stdout)
        self.assertIn("relying on it, no petter started", r.stdout)
        self.assertNotIn("failed to start", r.stdout)
        self.assertFalse((self.pet_dir / "wdt_pet.pid").exists())
        self.assertIn("BUSY: %s held by another owner" % busy_dev, self.log.read_text())
        self.assertNotIn("OPEN pid=", self.log.read_text())

    def test_petter_blocked_in_open_is_not_reported_started(self) -> None:
        fifo = self.tmp / "fifo_watchdog"
        os.mkfifo(fifo)
        self.fifos.append(fifo)
        # open(O_WRONLY) on a FIFO with no reader blocks: the petter is alive
        # but never logs OPEN within the wait. 2 s, not the board's 3 s: long
        # enough for python to install its SIGTERM handler on a slow runner
        # (a TERM before that just kills it before any open, also safe)
        r = self.wdt("start", FAKE_RWD="0", WDT_DEV=str(fifo), WDT_PET_START_POLLS="10")
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn("never logged OPEN and is still alive", r.stdout)
        self.assertNotIn("wdt_pet started", r.stdout)
        pidfile = self.pet_dir / "wdt_pet.pid"
        self.assertTrue(pidfile.exists(), "kept so 'stop' can still reach it")
        pid = int(pidfile.read_text())
        # the SIGTERM it already got must still end in a magic close once the
        # open completes: give it a reader and expect exactly 'V'
        fd = os.open(fifo, os.O_RDONLY | os.O_NONBLOCK)
        got = b""
        deadline = time.monotonic() + 10
        try:
            while b"V" not in got and time.monotonic() < deadline:
                try:
                    got += os.read(fd, 16)
                except BlockingIOError:
                    pass
                time.sleep(0.05)
        finally:
            os.close(fd)
        self.assertEqual(got, b"V")
        deadline = time.monotonic() + 10
        while "CLOSE-WITH-MAGIC" not in self.log.read_text() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertIn("CLOSE-WITH-MAGIC after 0 pets", self.log.read_text())
        self.assertIn("OPEN pid=%d " % pid, self.log.read_text())
        r = self.wdt("stop")
        self.assertRegex(r.stdout, r"wdt_pet (stopped )?pid=%d" % pid)
        self.assertFalse(pidfile.exists())


# --------------------------------------------------------------------------
@unittest.skipUnless(DASH or shutil.which("sh"), "a POSIX sh required")
class WDO_D_Installer(_ScriptHarness):

    def _wd(self) -> Path:
        return self.etc / "system.conf.d" / "90-lifetrac-watchdog.conf"

    def _rt(self) -> Path:
        return self.etc / "system" / "reboot.target.d" / "90-lifetrac-timeout.conf"

    def _assert_nothing_written(self) -> None:
        self.assertFalse(self.etc.exists() and any(self.etc.rglob("*")),
                         list(self.etc.rglob("*")) if self.etc.exists() else [])

    def test_install_check_remove_base(self) -> None:
        r = self.installer("check")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("L2_DROPINS=absent", r.stdout)

        r = self.installer("install", "--role", "base")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(self._wd().read_bytes(), WD_DROPIN.read_bytes())
        self.assertEqual(self._rt().read_bytes(), RT_DROPIN.read_bytes())
        for p in (self._wd(), self._rt()):
            self.assertEqual(stat.S_IMODE(p.stat().st_mode), 0o644)
        self.assertIn("sudo systemctl daemon-reload", r.stdout)   # printed, not run
        self.assertIn("whoever runs it", r.stdout)
        # a reload is more than daemon-reload (2026-10-17 review)
        self.assertIn("without --no-reload", r.stdout)
        # production arming of the base waits on PWR-1's OTA/shutdown checks
        self.assertIn("BASE: install for the PWR-1 checks", r.stdout)
        self.assertIn("staged", r.stdout)
        self.assertIn("ostree-finalize-staged", r.stdout)
        # no temp files left beside the drop-ins
        self.assertEqual(sorted(p.name for p in self._wd().parent.iterdir()),
                         ["90-lifetrac-watchdog.conf"])
        self.assertEqual(sorted(p.name for p in self._rt().parent.iterdir()),
                         ["90-lifetrac-timeout.conf"])

        r = self.installer("install", "--role=base")
        self.assertEqual(r.returncode, 0)
        self.assertEqual(r.stdout.count("unchanged"), 2)

        r = self.installer("check", FAKE_RWD="0")
        self.assertIn("L2_DROPINS=installed", r.stdout)
        self.assertIn("RuntimeWatchdogUSec=0", r.stdout)
        self.assertIn("has not re-read it", r.stdout)
        self.assertIn("sudo systemctl daemon-reload", r.stdout)
        self.assertNotIn("daemon-reexec", r.stdout)
        r = self.installer("check", FAKE_RWD="30s")
        self.assertIn("RuntimeWatchdogUSec=30s", r.stdout)
        self.assertNotIn("has not re-read it", r.stdout)
        self.assertNotIn("NOTE:", r.stdout)
        self.assertIn("reboot.target JobTimeoutUSec=3min", r.stdout)
        self.assertIn("%s held by: 1(" % self.env["WDT_DEV"], r.stdout)

        r = self.installer("remove")
        self.assertEqual(r.returncode, 0)
        self.assertIn("Not disarmed by this script", r.stdout)
        self.assertIn("sudo systemctl daemon-reload", r.stdout)
        self.assertFalse(self._wd().exists())
        self.assertFalse(self._rt().exists())
        r = self.installer("check")
        self.assertIn("L2_DROPINS=absent", r.stdout)
        self.assertNotIn("NOTE:", r.stdout)
        # removed but PID 1 not reloaded yet: still armed, and check says so
        r = self.installer("check", FAKE_RWD="30s")
        self.assertIn("armed (RuntimeWatchdogUSec=30s) without our drop-in", r.stdout)

        calls = self.calls_text()
        for forbidden in ("daemon-reexec", "daemon-reload", "reboot\n", "start", "restart"):
            self.assertNotIn(forbidden, calls)
        for line in calls.splitlines():
            self.assertTrue(line.startswith(("show ", "fuser ", "systemd-analyze cat-config")), line)

    def test_check_reports_partial_and_differs(self) -> None:
        self.assertEqual(self.installer("install", "--role", "base").returncode, 0)
        self._rt().unlink()
        self.assertIn("L2_DROPINS=partial", self.installer("check").stdout)
        self.assertEqual(self.installer("install", "--role", "base").returncode, 0)
        self._wd().write_text("[Manager]\nRuntimeWatchdogSec=0\n")
        r = self.installer("check")
        self.assertIn("DIFFERS", r.stdout)
        self.assertIn("L2_DROPINS=differs", r.stdout)
        # install repairs it
        self.assertEqual(self.installer("install", "--role", "base").returncode, 0)
        self.assertEqual(self._wd().read_bytes(), WD_DROPIN.read_bytes())

    def _run_copy(self, script: Path, *args: str) -> subprocess.CompletedProcess:
        return subprocess.run([DASH or "sh", str(script), *args], env=self.env,
                              capture_output=True, text=True, timeout=60)

    def test_check_without_the_repo_copy_is_unverified_not_differs(self) -> None:
        self.assertEqual(self.installer("install", "--role", "base").returncode, 0)
        # only the script was pushed to the board: nothing to compare against,
        # which must not read as "differs"
        lone = self.tmp / "lone dir"
        lone.mkdir()
        shutil.copy2(INSTALLER, lone / "install_watchdog.sh")
        r = self._run_copy(lone / "install_watchdog.sh", "check")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn("DIFFERS", r.stdout)
        self.assertEqual(r.stdout.count("cannot compare"), 2)
        self.assertIn("L2_DROPINS=present-unverified", r.stdout)
        # and install from there refuses instead of writing something else
        r = self._run_copy(lone / "install_watchdog.sh", "install", "--role", "base")
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn("push the whole x8_recovery folder", r.stderr)
        self.assertEqual(self._wd().read_bytes(), WD_DROPIN.read_bytes())
        # a symlink to the real script still finds systemd/ beside its target
        link = self.tmp / "link dir" / "iw.sh"
        link.parent.mkdir()
        link.symlink_to(INSTALLER)
        r = self._run_copy(link, "check")
        self.assertNotIn("DIFFERS", r.stdout)
        self.assertIn("L2_DROPINS=installed", r.stdout)

    def test_check_without_an_answer_is_unknown_not_off(self) -> None:
        # systemctl failing (not a systemd boot) or timing out (wedged PID 1)
        # gives no value; check must not read that as "not armed yet" and
        # steer the operator into reloading a wedged PID 1
        self.assertEqual(self.installer("install", "--role", "base").returncode, 0)
        r = self.installer("check", FAKE_RWD="__fail__")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("L2_DROPINS=installed", r.stdout)
        self.assertIn("armed state unknown", r.stdout)
        self.assertNotIn("has not re-read it", r.stdout)
        self.assertNotIn("daemon-reload", r.stdout)
        # and the reboot.target query is skipped (one 5 s wait, not two)
        self.assertNotIn("reboot.target", self.calls_text())

    def test_tractor_refused_without_flag(self) -> None:
        r = self.installer("install", "--role", "tractor")
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        self.assertIn("TRACTOR: refused", r.stderr)
        self.assertIn("--i-accept-tractor-prereqs", r.stderr)
        self.assertIn("stm32h7-program", r.stderr)
        for item in ("PWR-8", "PWR-9", "PWR-11"):
            self.assertIn(item, r.stderr)
        self._assert_nothing_written()
        r = self.installer("install", "--role", "tractor", "--i-accept-tractor-prereqs")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertTrue(self._wd().exists() and self._rt().exists())

    def test_bench_role_needs_the_bench_marker(self) -> None:
        # 2026-10-17 review: --role bench --bench-opt-in on a production
        # tractor (no marker) installed both drop-ins and never met the
        # tractor gate
        for extra in ((), ("--bench-opt-in",)):
            with self.subTest(extra=extra):
                r = self.installer("install", "--role", "bench", *extra)
                self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
                self.assertIn("not a bench board (no %s)" % self.bench_marker, r.stderr)
                self.assertIn("--role tractor", r.stderr)
                self._assert_nothing_written()

    def test_bench_needs_opt_in(self) -> None:
        self.bench_marker.parent.mkdir()
        self.bench_marker.write_text("fio ALL=(ALL) NOPASSWD: ALL\n")
        r = self.installer("install", "--role", "bench")
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        self.assertIn("BENCH WARNING", r.stdout)
        self.assertIn("--bench-opt-in", r.stderr)
        self._assert_nothing_written()
        r = self.installer("install", "--role", "bench", "--bench-opt-in")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("BENCH WARNING", r.stdout)
        self.assertIn("H747 bank 1", r.stdout)   # the bench tractor's cost too
        self.assertTrue(self._wd().exists() and self._rt().exists())

    def test_bench_marker_overrides_the_role(self) -> None:
        # the only base X8 is a bench board: the sudoers marker, not the typed
        # role, decides whether the bench opt-in is needed
        self.bench_marker.parent.mkdir()
        self.bench_marker.write_text("fio ALL=(ALL) NOPASSWD: ALL\n")
        for args in (("--role", "base"),
                     ("--role", "tractor", "--i-accept-tractor-prereqs")):
            with self.subTest(args=args):
                r = self.installer("install", *args)
                self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
                self.assertIn("BENCH WARNING", r.stdout)
                self.assertIn("%s marks this as a bench board" % self.bench_marker, r.stderr)
                self.assertIn("--bench-opt-in", r.stderr)
                self._assert_nothing_written()
        # the opt-in does not stand in for the tractor gate
        r = self.installer("install", "--role", "tractor", "--bench-opt-in")
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        self.assertIn("TRACTOR: refused", r.stderr)
        self._assert_nothing_written()
        r = self.installer("install", "--role", "base", "--bench-opt-in")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("BENCH WARNING", r.stdout)
        self.assertTrue(self._wd().exists() and self._rt().exists())

    def test_install_notes_a_live_petter(self) -> None:
        # a live wdt_pet petter will race the next reload: say so. Not
        # harmless (2026-10-17 review): systemd retries the open, but after an
        # earlier arm/disarm in the same boot PID 1 busy-loops on EBUSY until
        # the petter closes, so the note says to stop the petter first
        self.pet_dir.mkdir()
        pidfile = self.pet_dir / "wdt_pet.pid"
        pidfile.write_text("%d\n" % os.getpid())     # alive, and not a petter
        r = self.installer("install", "--role", "base")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("NOTE: wdt_pet.sh's petter (pid %d) holds" % os.getpid(), r.stdout)
        note = next(ln for ln in r.stdout.splitlines() if ln.startswith("NOTE:"))
        self.assertIn("'bash wdt_pet.sh stop'", note)
        self.assertIn("before any reload", note)
        pidfile.write_text("999999999\n")             # stale: no note
        r = self.installer("install", "--role", "base")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertNotIn("NOTE:", r.stdout)

    def test_usage_errors(self) -> None:
        for args in (("install",), ("install", "--role", "plow"), ("install", "--bogus"),
                     ("frobnicate",), ()):
            with self.subTest(args=args):
                r = self.installer(*args)
                self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
                self.assertIn("usage:", r.stderr)
        self._assert_nothing_written()


# --------------------------------------------------------------------------
class WDO_E_ReadOnlyTripwires(unittest.TestCase):

    @staticmethod
    def _code(path: Path) -> str:
        """The script without comment-only lines."""
        return "\n".join(l for l in path.read_text().splitlines()
                         if not l.lstrip().startswith("#"))

    def test_capture_stays_read_only(self) -> None:
        code = self._code(CAPTURE)
        for pat in (r"\bi2c(get|set|dump|transfer)\b", r">\s*/proc/sysrq-trigger",
                    r">\s*/proc/sys/", r">\s*/dev/watchdog", r"\bfw_setenv\b",
                    r"\becho\s+[a-z]\s*>\s*/proc"):
            self.assertIsNone(re.search(pat, code), pat)
        header = CAPTURE.read_text().split("set -u", 1)[0]
        for needle in ("opening /dev/watchdog0", "/proc/sysrq-trigger", "i2cget"):
            self.assertIn(needle, header)
        # the new facts are captured
        for needle in ("RuntimeWatchdogUSec", "reboot.target", "/proc/sys/kernel",
                       "pmic@4b", "pwrkey", "HandlePowerKey", "fw_printenv bootcount"):
            self.assertIn(needle, code)

    def test_installer_and_petter_never_write_the_device(self) -> None:
        for path in (INSTALLER, WDT_PET):
            code = self._code(path)
            self.assertIsNone(re.search(r">\s*\"?\$\{?WDT_DEV", code), path.name)
            self.assertIsNone(re.search(r">\s*/dev/watchdog", code), path.name)
            self.assertNotIn("sysrq-trigger", code)
        # the installer prints the reexec command; no line runs it (the fake
        # systemctl call log in WDO_D checks the same thing by behaviour)
        self.assertNotRegex(self._code(INSTALLER),
                            re.compile(r"^\s*(q\s+)?systemctl\s+(daemon-re|reboot)", re.M))

    def test_shell_syntax(self) -> None:
        jobs = [(BASH, WDT_PET), (BASH, FLASH_PIPELINE)]
        sh = DASH or shutil.which("sh")
        jobs += [(sh, INSTALLER), (sh, CAPTURE)]
        for shell, path in jobs:
            if not shell:
                continue
            with self.subTest(script=path.name, shell=shell):
                r = subprocess.run([shell, "-n", str(path)], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stderr)


if __name__ == "__main__":
    unittest.main()
