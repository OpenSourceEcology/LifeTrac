"""bench_tools legs: SIL tests for the 2026-10-22 kit fixes (drain + evidence guard).

BLEG_A_EvidenceGuard
    lib/bench_env.sh, sourced in bash: bench_evidence_guard / bench_evidence_dir
    REFUSE an EVIDENCE_DIR that already holds files tracked by git (the committed
    bench-evidence/RS_13_range_edge_2026-10-19/legs of this checkout, in C:/...,
    /c/... and backslash form on Windows; a folder staged in a throwaway git repo
    everywhere) and create nothing there; BENCH_EVIDENCE_APPEND=1 in the
    environment allows the append with a NOTE, while the same line in the settings
    file is ignored with a WARN; a new folder in a checkout and a folder outside
    any checkout pass and are created; a YYYY-MM-DD in the path that is not today
    is a WARNING, never a refusal; --warn-only turns the refusal into a WARNING;
    a warning prints once per process tree, a refusal every time.
BLEG_B_LegPrepDrain
    leg_prep.sh --print-harness (touches no board): -StopDrainS 4 on image_bw250
    (FHSS), 2 on image_bw500 (DTS), the same for range-edge tags; --stop-drain S
    is honoured (0 < S <= 30) in the PLAN line, the printed harness line and the
    .ps1 wrapper, with a NOTE to fly the round's other codec with the same value;
    0, 31, -1, '', '.5' and 'abc' are refused. The preview shows a committed
    EVIDENCE_DIR's REFUSED verdict and writes nothing into that folder. On a
    deliberate append (BENCH_EVIDENCE_APPEND=1 on a committed folder) the printed
    leg2d_switch / link_sample / leg_post lines carry the same override. Every
    case runs under the stubs below and must log no adb / ssh / scp call.
BLEG_C_BoardOrdering
    With stub adb / ssh / scp first on PATH (every call logged; the run aborts if
    the stubs do not shadow the real tools): leg_prep.sh (the real prep) and
    stage_boards.sh refuse a committed folder with exit 7 before ANY adb call and
    write nothing there; leg_post.sh only WARNS, writes the warning into its
    transcript and still parks both radios (with BENCH_EVIDENCE_APPEND=1: a NOTE,
    and it parks).
BLEG_D_Syntax
    ``bash -n`` on lib/bench_env.sh and every legs/*.sh.

Hermetic: no real adb, ssh or board, nothing is written into this checkout (the
committed folder is only checked, never written; writable folders live in temp
dirs and a throwaway ``git init`` repo whose files are staged, not committed).
On Windows the tests need Git for Windows' bash (Git\\bin\\bash.exe; never the
WSL bash in System32) and skip without it; elsewhere any bash.
"""
from __future__ import annotations

import datetime
import os
import shutil
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_DC = _BS.parent                                     # DESIGN-CONTROLLER/
_BT = _DC / "firmware" / "x8_lora_bootloader_helper" / "bench_tools"
_LEGS = _BT / "legs"
BENCH_ENV_SH = _BT / "lib" / "bench_env.sh"
EXAMPLE_ENV = _BT / "bench.env.example"
LEG_PREP = _LEGS / "leg_prep.sh"
LEG_POST = _LEGS / "leg_post.sh"
STAGE_BOARDS = _LEGS / "stage_boards.sh"
COMMITTED = _DC / "bench-evidence" / "RS_13_range_edge_2026-10-19" / "legs"


def _find_bash() -> str | None:
    if os.name != "nt":
        return shutil.which("bash")
    # Git for Windows: Git\bin\bash.exe puts its /usr/bin and /mingw64/bin on PATH.
    cands = []
    git = shutil.which("git")
    if git:
        cands += [p / "bin" / "bash.exe" for p in Path(git).resolve().parents]
    for var in ("ProgramW6432", "ProgramFiles", "ProgramFiles(x86)"):
        if os.environ.get(var):
            cands.append(Path(os.environ[var]) / "Git" / "bin" / "bash.exe")
    for c in cands:
        if c.is_file():
            return str(c)
    return None


BASH = _find_bash()
_SKIP = "needs bash (Git for Windows' bash on Windows)"

# Puts the stub dir first on PATH and refuses to go on unless adb, ssh and scp
# resolve to the stubs -- a real adb would reach the bench boards.
_STUB_WRAP = r'''
d=$1; shift
if command -v cygpath >/dev/null 2>&1; then d=$(cygpath -u "$d"); fi
export PATH="$d:$PATH"
for t in adb ssh scp; do
  [ "$(command -v "$t")" = "$d/$t" ] || { echo "SIL: stub $t is not first on PATH ($(command -v "$t"))" >&2; exit 99; }
done
exec bash "$@"
'''
_STUB_ADB = '''#!/bin/sh
printf '%s\\n' "$*" >> "$SIL_ADB_LOG"
case "$*" in
  *radio_park.py*) echo "PARK_OK radio_state=0x80 (SIL stub)" ;;
  *__UART_CHK_DONE__*) printf '\\n__UART_CHK_DONE__\\n' ;;
esac
exit 0
'''
_STUB_FAIL = '''#!/bin/sh
printf '%s %s\\n' "$(basename "$0")" "$*" >> "$SIL_ADB_LOG"
exit 1
'''


def _posix(p: Path | str) -> str:
    return Path(p).as_posix()


@unittest.skipUnless(BASH, _SKIP)
class _BashCase(unittest.TestCase):
    """Temp dirs, a clean environment, stub adb / ssh / scp and the bash runners."""

    @classmethod
    def setUpClass(cls) -> None:
        cp = subprocess.run([BASH, "-c", "date +%Y-%m-%d"], capture_output=True, text=True, timeout=60)
        cls.today = cp.stdout.strip() or datetime.date.today().isoformat()

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="bleg_sil_"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.scratch = self.tmp / "scratch"
        self.scratch.mkdir()
        self.stub = self.tmp / "stub"
        self.stub.mkdir()
        for name, body in (("adb", _STUB_ADB), ("ssh", _STUB_FAIL), ("scp", _STUB_FAIL)):
            p = self.stub / name
            p.write_text(body, encoding="ascii", newline="\n")
            p.chmod(p.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        self.adb_log = self.tmp / "adb_calls.txt"

    def env(self, **kw: str) -> dict:
        env = {k: v for k, v in os.environ.items()
               if not k.startswith(("BENCH_", "DTS_", "SIL_"))
               and k not in ("EVIDENCE_DIR", "ARCHIVE_DIR", "PC_HOST", "BASE_SERIAL", "TRACTOR_SERIAL",
                             "BASE_TRANSPORT", "BASE_HOST", "BASE_USER", "BASE_SSH_USER", "STAMP_LOG")}
        env.update(BENCH_ENV_FILE=_posix(EXAMPLE_ENV), BENCH_SCRATCH=_posix(self.scratch),
                   BASE_TRANSPORT="adb", PYTHONIOENCODING="utf-8")
        env.update(kw)
        return env

    def run_bash(self, args: list, env: dict, timeout: int = 120) -> subprocess.CompletedProcess:
        return subprocess.run([BASH, *args], env=env, capture_output=True, text=True,
                              encoding="utf-8", errors="replace", timeout=timeout)

    def lib(self, code: str, **kw: str) -> subprocess.CompletedProcess:
        """Source bench_env.sh, then run CODE, in one bash."""
        return self.run_bash(["-c", '. "$1" || exit 98; eval "$2"', "_", _posix(BENCH_ENV_SH), code], self.env(**kw))

    def run_stubbed(self, script: Path, *args: str, timeout: int = 300, **kw: str) -> subprocess.CompletedProcess:
        """bash SCRIPT ARGS with the stubs first on PATH; it never starts unless they shadow the real tools."""
        env = self.env(SIL_ADB_LOG=_posix(self.adb_log), PC_HOST="192.168.1.50")
        env.update(kw)
        cp = self.run_bash(["-c", _STUB_WRAP, "_", _posix(self.stub), _posix(script), *args], env, timeout=timeout)
        self.assertNotEqual(99, cp.returncode, cp.stderr)
        return cp

    def adb_calls(self) -> list:
        """Every adb / ssh / scp call the stubs logged."""
        if not self.adb_log.exists():
            return []
        return [ln for ln in self.adb_log.read_text(encoding="utf-8", errors="replace").splitlines() if ln.strip()]

    def git_repo_with_tracked(self) -> Path:
        """A throwaway checkout whose bench-evidence/OLD_2026-01-01/legs holds a staged (tracked) file."""
        repo = self.tmp / "repo"
        legs = repo / "bench-evidence" / "OLD_2026-01-01" / "legs"
        legs.mkdir(parents=True)
        (legs / "legX_report.txt").write_text("committed\n", encoding="ascii", newline="\n")
        subprocess.run(["git", "init", "-q", str(repo)], check=True, capture_output=True, timeout=60)
        subprocess.run(["git", "-C", str(repo), "add", "."], check=True, capture_output=True, timeout=60)
        return legs


class BLEG_A_EvidenceGuard(_BashCase):

    def committed(self) -> Path:
        cp = subprocess.run(["git", "-C", str(_DC), "ls-files", "--", _posix(COMMITTED)],
                            capture_output=True, text=True, timeout=60)
        if cp.returncode != 0 or not cp.stdout.strip():
            self.skipTest("bench-evidence/RS_13_range_edge_2026-10-19/legs is not tracked in this checkout")
        return COMMITTED

    def test_committed_folder_of_this_checkout_refused(self) -> None:
        d = self.committed()
        before = sorted(p.name for p in d.iterdir())
        forms = [_posix(d)]
        if os.name == "nt":
            p = _posix(d)
            forms += ["/" + p[0].lower() + p[2:], str(d)]          # /c/Users/... and C:\Users\...
        for form in forms:
            with self.subTest(form=form):
                cp = self.lib('bench_evidence_dir; echo "rc=$?"', EVIDENCE_DIR=form)
                self.assertIn("rc=1", cp.stdout)
                self.assertIn("REFUSED", cp.stderr)
                self.assertIn("tracked by git", cp.stderr)
                self.assertIn("BENCH_EVIDENCE_APPEND=1", cp.stderr)
                self.assertNotIn(_posix(d) + "\n", cp.stdout, "a refused folder must not be printed")
        self.assertEqual(before, sorted(p.name for p in d.iterdir()))

    def test_append_from_the_environment_allowed(self) -> None:
        d = self.committed()
        cp = self.lib('bench_evidence_guard; echo "rc=$?"', EVIDENCE_DIR=_posix(d), BENCH_EVIDENCE_APPEND="1")
        self.assertIn("rc=0", cp.stdout)
        self.assertIn("NOTE: BENCH_EVIDENCE_APPEND=1", cp.stderr)
        self.assertNotIn("REFUSED", cp.stderr)

    def test_append_in_the_settings_file_is_ignored(self) -> None:
        d = self.committed()
        env_file = self.tmp / "bench.env"
        env_file.write_text(EXAMPLE_ENV.read_text(encoding="utf-8") + "\nBENCH_EVIDENCE_APPEND=1\n",
                            encoding="utf-8", newline="\n")
        cp = self.lib('bench_evidence_guard; echo "rc=$?"', EVIDENCE_DIR=_posix(d), BENCH_ENV_FILE=_posix(env_file))
        self.assertIn("rc=1", cp.stdout)
        self.assertIn("BENCH_EVIDENCE_APPEND in", cp.stderr)
        self.assertIn("is ignored", cp.stderr)
        self.assertIn("REFUSED", cp.stderr)

    def test_other_values_do_not_append(self) -> None:
        legs = self.git_repo_with_tracked()
        for val in ("yes", "true", "0", "2"):
            with self.subTest(val=val):
                cp = self.lib('bench_evidence_guard; echo "rc=$?"', EVIDENCE_DIR=_posix(legs), BENCH_EVIDENCE_APPEND=val)
                self.assertIn("rc=1", cp.stdout)

    def test_tracked_folder_of_any_checkout_refused_and_untouched(self) -> None:
        legs = self.git_repo_with_tracked()
        cp = self.lib('bench_evidence_dir; echo "rc=$?"', EVIDENCE_DIR=_posix(legs))
        self.assertIn("rc=1", cp.stdout)
        self.assertIn("1 file(s) tracked by git", cp.stderr)
        self.assertIn("every other script of the leg with BENCH_EVIDENCE_APPEND=1", cp.stderr)
        self.assertEqual(["legX_report.txt"], sorted(p.name for p in legs.iterdir()))
        # a subfolder of it that does not exist yet holds nothing tracked: allowed
        sub = legs / "new_sub"
        cp = self.lib('bench_evidence_dir; echo "rc=$?"', EVIDENCE_DIR=_posix(sub))
        self.assertIn("rc=0", cp.stdout)

    def test_warn_only_never_refuses(self) -> None:
        legs = self.git_repo_with_tracked()
        cp = self.lib('bench_evidence_dir --warn-only; echo "rc=$?"', EVIDENCE_DIR=_posix(legs))
        self.assertIn("rc=0", cp.stdout)
        self.assertIn(_posix(legs), cp.stdout)
        self.assertIn("WARNING", cp.stderr)
        self.assertIn("committed evidence record", cp.stderr)
        self.assertIn("If this append was deliberate, run every script of the leg with BENCH_EVIDENCE_APPEND=1", cp.stderr)
        self.assertNotIn("REFUSED", cp.stderr)

    def test_new_folder_in_a_checkout_allowed_and_created(self) -> None:
        legs = self.git_repo_with_tracked()
        new = legs.parent.parent / ("NEW_%s" % self.today) / "legs"
        cp = self.lib('bench_evidence_dir; echo "rc=$?"', EVIDENCE_DIR=_posix(new))
        self.assertIn("rc=0", cp.stdout)
        self.assertIn(_posix(new), cp.stdout)
        self.assertTrue(new.is_dir())
        self.assertEqual("", cp.stderr.strip())

    def test_new_folder_in_this_checkout_allowed_without_creating(self) -> None:
        new = _DC / "bench-evidence" / ("SIL_never_made_%s" % self.today) / "legs"
        self.assertFalse(new.exists())
        cp = self.lib('bench_evidence_guard; echo "rc=$? n=$(bench_evidence_tracked "$EVIDENCE_DIR")"',
                      EVIDENCE_DIR=_posix(new))
        self.assertIn("rc=0 n=0", cp.stdout)
        self.assertFalse(new.exists(), "the guard alone must create nothing")

    def test_folder_outside_any_checkout_allowed(self) -> None:
        outside = self.tmp / "outside" / ("ev_%s" % self.today) / "legs"
        probe = subprocess.run(["git", "-C", str(self.tmp), "rev-parse", "--is-inside-work-tree"],
                               capture_output=True, text=True, timeout=60)
        if probe.returncode == 0:
            self.skipTest("the temp dir is inside a git checkout")
        cp = self.lib('bench_evidence_dir; echo "rc=$?"', EVIDENCE_DIR=_posix(outside))
        self.assertIn("rc=0", cp.stdout)
        self.assertTrue(outside.is_dir())
        self.assertNotIn("WARNING", cp.stderr)

    def test_git_that_cannot_answer_refuses(self) -> None:
        # a broken worktree link ("not a git repository: <path>", without the
        # "(or any" of a folder outside every checkout) and a git that reports
        # dubious ownership must not read as "nothing committed"
        broken = self.tmp / ("broken_%s" % self.today)
        broken.mkdir()
        (broken / ".git").write_text("gitdir: %s\n" % _posix(self.tmp / "gone" / ".git" / "worktrees" / "x"),
                                     encoding="ascii", newline="\n")
        dubious = self.tmp / "gitstub"
        dubious.mkdir()
        g = dubious / "git"
        g.write_text("#!/bin/sh\necho \"fatal: detected dubious ownership in repository at '/x'\" >&2\nexit 128\n",
                     encoding="ascii", newline="\n")
        g.chmod(g.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        cases = [("broken link", broken / "legs", ""),
                 ("dubious ownership", self.tmp / ("ev_%s" % self.today) / "legs",
                  # set in bash, in its own form (C:/... would split at the drive colon)
                  'PATH="$(cygpath -u "%s" 2>/dev/null || echo "%s"):$PATH"; '
                  % (_posix(dubious), _posix(dubious)))]
        for what, d, pre in cases:
            with self.subTest(what=what):
                cp = self.lib(pre + 'bench_evidence_dir; echo "rc=$?"', EVIDENCE_DIR=_posix(d))
                self.assertIn("rc=1", cp.stdout, cp.stderr)
                self.assertIn("REFUSED: git could not say", cp.stderr)
                self.assertFalse(d.exists(), "a refused folder must not be created")
                cp = self.lib(pre + 'bench_evidence_guard; echo "rc=$?"', EVIDENCE_DIR=_posix(d),
                              BENCH_EVIDENCE_APPEND="1")
                self.assertIn("rc=0", cp.stdout)
                self.assertIn("NOTE: BENCH_EVIDENCE_APPEND=1 -- git could not say", cp.stderr)
                cp = self.lib(pre + 'bench_evidence_guard --warn-only; echo "rc=$?"', EVIDENCE_DIR=_posix(d))
                self.assertIn("rc=0", cp.stdout)
                self.assertIn("WARNING: git could not say", cp.stderr)
                self.assertNotIn("REFUSED", cp.stderr)

    def test_date_warning(self) -> None:
        old = self.tmp / "RS_13_x_2020-01-01" / "legs"
        cp = self.lib('bench_evidence_dir; echo "rc=$?"', EVIDENCE_DIR=_posix(old))
        self.assertIn("rc=0", cp.stdout, "a stale date only warns")
        self.assertIn("WARNING", cp.stderr)
        self.assertIn("dated 2020-01-01", cp.stderr)
        self.assertIn(self.today, cp.stderr)
        for d in (self.tmp / ("RS_13_x_%s" % self.today) / "legs", self.tmp / "undated" / "legs",
                  self.tmp / "2020-01-01_old" / ("now_%s" % self.today)):        # the LAST date counts
            with self.subTest(d=d):
                cp = self.lib('bench_evidence_dir; echo "rc=$?"', EVIDENCE_DIR=_posix(d))
                self.assertIn("rc=0", cp.stdout)
                self.assertNotIn("WARNING", cp.stderr)

    def test_warning_once_per_process_tree_refusal_every_time(self) -> None:
        old = self.tmp / "RS_13_x_2020-01-01" / "legs"
        cp = self.lib('bench_evidence_guard; bench_evidence_guard; bash -c \'. "$0"; bench_evidence_guard\' "$BENCH_LIB_DIR/bench_env.sh"; '
                      'echo "msg=[$BENCH_EVIDENCE_MSG]"', EVIDENCE_DIR=_posix(old))
        self.assertEqual(1, cp.stderr.count("dated 2020-01-01"))
        self.assertIn("msg=[]", cp.stdout, "the second call has nothing new to say")
        legs = self.git_repo_with_tracked()
        cp = self.lib('bench_evidence_guard; bench_evidence_guard; echo "msg=[$BENCH_EVIDENCE_MSG]"', EVIDENCE_DIR=_posix(legs))
        self.assertEqual(2, cp.stderr.count("REFUSED"))
        self.assertIn("msg=[REFUSED", cp.stdout)


class BLEG_B_LegPrepDrain(_BashCase):

    def prep(self, *args: str, **kw: str) -> subprocess.CompletedProcess:
        """leg_prep.sh ARGS --print-harness under the stubs: should option parsing or the
        preview ever regress into the real prep, no real adb is reached, and it fails here."""
        env = dict(DTS_CARRIER_HZ="905250000", DTS_CARRIER_DATE=self.today,
                   EVIDENCE_DIR=_posix(self.tmp / ("ev_%s" % self.today) / "legs"))
        env.update(kw)
        cp = self.run_stubbed(LEG_PREP, *args, "--print-harness", **env)
        self.assertEqual([], self.adb_calls(), "a preview (or a refused option) must not touch a board")
        return cp

    def wrapper(self, leg: str) -> str:
        return (self.scratch / ("leg%s_harness.ps1" % leg)).read_text(encoding="utf-8")

    def test_defaults_by_profile(self) -> None:
        for leg, prof, want, why in (("2b", "image_bw250", "4", "FHSS default"),
                                     ("2a", "image_bw500", "2", "DTS default"),
                                     ("leak_fhss", "", "4", "FHSS default"),
                                     ("A2c", "", "2", "DTS default")):
            with self.subTest(leg=leg):
                cp = self.prep(*([leg, prof] if prof else [leg]))
                self.assertEqual(0, cp.returncode, cp.stderr)
                self.assertIn("-StopDrainS %s (%s)" % (want, why), cp.stdout)          # the PLAN line
                self.assertIn("-DurationS", cp.stdout)
                self.assertIn(" -StopDrainS %s -SynthFps" % want, cp.stdout)          # the harness line
                self.assertIn(" -StopDrainS %s -SynthFps" % want, self.wrapper(leg))
                self.assertNotIn("--stop-drain", cp.stdout)
        self.assertFalse((self.tmp / ("ev_%s" % self.today)).exists(), "a preview creates no evidence folder")

    def test_stop_drain_option_honoured(self) -> None:
        for val in ("6", "0.5", "30", "2.5"):
            with self.subTest(val=val):
                cp = self.prep("2b", "image_bw250", "--stop-drain", val)
                self.assertEqual(0, cp.returncode, cp.stderr)
                self.assertIn("-StopDrainS %s (--stop-drain; default 4)" % val, cp.stdout)
                self.assertIn(" -StopDrainS %s -SynthFps" % val, self.wrapper("2b"))
                self.assertIn("fly the other codec of this round with the same --stop-drain %s" % val, cp.stdout)
        cp = self.prep("2a", "image_bw500", "--stop-drain", "4")
        self.assertEqual(0, cp.returncode, cp.stderr)
        self.assertIn(" -StopDrainS 4 -SynthFps", self.wrapper("2a"))
        cp = self.prep("2b", "image_bw250", "--stop-drain", "4")                   # = the default: no NOTE
        self.assertNotIn("fly the other codec", cp.stdout)

    def test_bad_values_refused(self) -> None:
        for val in ("0", "0.0", "31", "30.5", "-1", "", ".5", "abc", "4s", "1e1"):
            with self.subTest(val=val):
                cp = self.prep("2b", "image_bw250", "--stop-drain", val)
                self.assertNotEqual(0, cp.returncode)
                self.assertNotIn("harness wrapper", cp.stdout)
                if val:
                    self.assertIn("--stop-drain must be a number", cp.stderr)

    def test_usage_says_both_codecs_same_value(self) -> None:
        cp = self.run_stubbed(LEG_PREP, "--help")
        self.assertEqual(0, cp.returncode)
        self.assertIn("--stop-drain S", cp.stdout)
        self.assertIn("SAME", cp.stdout)
        self.assertIn("override on EVERY script of the leg", cp.stdout)
        self.assertEqual([], self.adb_calls())

    def test_preview_shows_a_committed_folder_refusal(self) -> None:
        legs = self.git_repo_with_tracked()
        cp = self.prep("2b", "image_bw250", EVIDENCE_DIR=_posix(legs))
        self.assertEqual(0, cp.returncode, cp.stderr)
        self.assertIn("REFUSED", cp.stdout)
        self.assertIn("stops at that REFUSED line, exit 7", cp.stdout)
        self.assertIn(" -StopDrainS 4 -SynthFps", self.wrapper("2b"))
        self.assertNotIn("BENCH_EVIDENCE_APPEND=1 bash", cp.stdout, "no append was asked for")
        self.assertEqual(["legX_report.txt"], sorted(p.name for p in legs.iterdir()))

    def test_deliberate_append_carried_to_the_follow_up_lines(self) -> None:
        # 10-22 review: the printed follow-ups run in new shells; without the override
        # leg2d_switch.sh and link_sample.sh refuse the committed folder mid-leg.
        legs = self.git_repo_with_tracked()
        cp = self.prep("2d", "image_bw500", EVIDENCE_DIR=_posix(legs), BENCH_EVIDENCE_APPEND="1")
        self.assertEqual(0, cp.returncode, cp.stderr)
        self.assertIn("NOTE: BENCH_EVIDENCE_APPEND=1", cp.stdout)
        self.assertNotIn("REFUSED", cp.stdout)
        for script in ("leg2d_switch.sh", "link_sample.sh", "leg_post.sh"):
            with self.subTest(script=script):
                lines = [ln for ln in cp.stdout.splitlines() if script + '" 2d' in ln]
                self.assertEqual(1, len(lines), cp.stdout)
                self.assertIn('BENCH_EVIDENCE_APPEND=1 bash "', lines[0])
        self.assertEqual(["legX_report.txt"], sorted(p.name for p in legs.iterdir()))
        # no append in effect (a new folder, with or without the override): plain lines
        for kw in ({}, {"BENCH_EVIDENCE_APPEND": "1"}):
            with self.subTest(env=kw):
                cp = self.prep("2d", "image_bw500", **kw)
                self.assertEqual(0, cp.returncode, cp.stderr)
                self.assertIn('link_sample.sh" 2d', cp.stdout)
                self.assertNotIn("BENCH_EVIDENCE_APPEND=1 bash", cp.stdout)

    def test_range_edge_evidence_note_kept(self) -> None:
        cp = self.prep("leak_fhss")
        self.assertIn("NOTE: EVIDENCE_DIR is", cp.stdout)
        self.assertIn("RS_13_range_edge_<date>/legs", cp.stdout)


class BLEG_C_BoardOrdering(_BashCase):

    def fake_archive(self) -> Path:
        arch = self.tmp / "radio_monitor_sil"
        arch.mkdir()
        (arch / "params.txt").write_text("reg_profile=1\nduration_s=10\nreactive_fire=0\nlog_window=after_stop\n",
                                         encoding="ascii", newline="\n")
        for f in ("rx_daemon.log", "tx_daemon.log"):
            (arch / f).write_text("", encoding="ascii")
        return arch

    def test_leg_prep_refuses_before_any_board_call(self) -> None:
        legs = self.git_repo_with_tracked()
        cp = self.run_stubbed(LEG_PREP, "2b", "image_bw250", "--no-scene-check", EVIDENCE_DIR=_posix(legs))
        self.assertEqual(7, cp.returncode, cp.stdout + cp.stderr)
        self.assertIn("REFUSED", cp.stderr)
        self.assertEqual([], self.adb_calls(), "no board may be touched before the refusal")
        self.assertEqual(["legX_report.txt"], sorted(p.name for p in legs.iterdir()))

    def test_stage_boards_refuses_before_any_board_call(self) -> None:
        legs = self.git_repo_with_tracked()
        cp = self.run_stubbed(STAGE_BOARDS, EVIDENCE_DIR=_posix(legs))
        self.assertEqual(7, cp.returncode, cp.stdout + cp.stderr)
        self.assertIn("REFUSED", cp.stderr)
        self.assertEqual([], self.adb_calls(), "a refusal must never leave a board half-staged")
        self.assertEqual(["legX_report.txt"], sorted(p.name for p in legs.iterdir()))

    def test_leg_post_warns_and_still_parks(self) -> None:
        legs = self.git_repo_with_tracked()
        arch = self.fake_archive()
        cp = self.run_stubbed(LEG_POST, "sil", _posix(arch), EVIDENCE_DIR=_posix(legs))
        out = cp.stdout + cp.stderr
        self.assertEqual(0, cp.returncode, out)
        self.assertIn("WARNING: EVIDENCE_DIR=", out)
        self.assertIn("committed evidence record", out)
        self.assertNotIn("REFUSED", out)
        self.assertIn("park both radios (last)", out)
        park = (legs / "legsil_park.txt").read_text(encoding="utf-8")
        self.assertIn("base:", park)
        self.assertIn("tractor:", park)
        self.assertEqual(2, park.count("PARK_OK"))
        self.assertEqual(2, sum("radio_park.py" in c for c in self.adb_calls()))
        transcript = (legs / "legsil_post_transcript.txt").read_text(encoding="utf-8", errors="replace")
        self.assertIn("committed evidence record", transcript)

    def test_leg_post_deliberate_append_notes_and_parks(self) -> None:
        # the line leg_prep prints on a deliberate append: a NOTE, not the move-them-out WARNING
        legs = self.git_repo_with_tracked()
        arch = self.fake_archive()
        cp = self.run_stubbed(LEG_POST, "sil", _posix(arch), EVIDENCE_DIR=_posix(legs), BENCH_EVIDENCE_APPEND="1")
        out = cp.stdout + cp.stderr
        self.assertEqual(0, cp.returncode, out)
        self.assertIn("NOTE: BENCH_EVIDENCE_APPEND=1", out)
        self.assertNotIn("move them out", out)
        self.assertIn("park both radios (last)", out)
        self.assertEqual(2, sum("radio_park.py" in c for c in self.adb_calls()))


class BLEG_D_Syntax(unittest.TestCase):

    @unittest.skipUnless(BASH, _SKIP)
    def test_bash_n(self) -> None:
        scripts = [BENCH_ENV_SH, _BT / "lib" / "env_file.sh", *sorted(_LEGS.glob("*.sh"))]
        self.assertIn(LEG_PREP, scripts)
        for s in scripts:
            with self.subTest(script=s.name):
                cp = subprocess.run([BASH, "-n", _posix(s)], capture_output=True, text=True, timeout=60)
                self.assertEqual(0, cp.returncode, cp.stderr)


if __name__ == "__main__":
    unittest.main()
