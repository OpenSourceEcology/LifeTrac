"""Tractor X8 CPU-only bench helper (no devices, no radio, /tmp/lifetrac_bench only).

  py -3 x8.py push            # mkdir /tmp/lifetrac_bench and push the bench + abold/ + abnew/
  py -3 x8.py run <args...>   # run `python3 <args>` in the tractor image, cwd /b = /tmp/lifetrac_bench
  py -3 x8.py pull <file>     # pull /tmp/lifetrac_bench/<file> here
  py -3 x8.py clean           # rm -rf /tmp/lifetrac_bench
"""
import os
import subprocess
import sys

SERIAL = "2E2C1209DABC240B"
REMOTE = "/tmp/lifetrac_bench"
HERE = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")
ENV = dict(os.environ, MSYS_NO_PATHCONV="1")


def adb(*args, check=True):
    cmd = ["adb", "-s", SERIAL] + list(args)
    r = subprocess.run(cmd, env=ENV, capture_output=True, text=True)
    if r.stdout:
        sys.stdout.write(r.stdout)
    if r.stderr:
        sys.stdout.write(r.stderr)
    if check and r.returncode != 0:
        raise SystemExit(f"adb failed: {' '.join(args)}")
    return r


def main():
    a = sys.argv[1:]
    if a[0] == "push":
        adb("shell", f"mkdir -p {REMOTE}/abold {REMOTE}/abnew")
        for f in ("bench.py", "ab.py", "exact_check.py", "order_check.py", "sources.npz"):
            adb("push", f"{HERE}/{f}", f"{REMOTE}/")
        for d in ("abold", "abnew"):
            for f in ("__init__.py", "encode_vector.py", "vector_extract.py", "vs1_codec.py"):
                adb("push", f"{HERE}/{d}/{f}", f"{REMOTE}/{d}/")
        adb("shell", f"rm -rf {REMOTE}/abold/__pycache__ {REMOTE}/abnew/__pycache__; ls -la {REMOTE} {REMOTE}/abold {REMOTE}/abnew")
    elif a[0] == "run":
        args = " ".join(a[1:])
        adb("shell", f'echo fio | sudo -S -p "" docker run --rm -v {REMOTE}:/b -w /b '
                     f'-e PYTHONIOENCODING=utf-8 -e PYTHONDONTWRITEBYTECODE=1 --entrypoint python3 lifetrac-tractor-x8:latest {args}')
    elif a[0] == "pushtests":
        wt = ("C:/Users/dorkm/Documents/GitHub/LifeTrac/.claude/worktrees/wf_adf2ad4c-341-1/"
              "LifeTrac-v25/DESIGN-CONTROLLER")
        pkg = "firmware/tractor_x8/x8_image_pipeline"
        adb("shell", f"mkdir -p {REMOTE}/DC/base_station/tests {REMOTE}/DC/{pkg}")
        adb("push", f"{wt}/base_station/tests/test_vector_fastpaths.py", f"{REMOTE}/DC/base_station/tests/")
        for f in ("__init__.py", "encode_vector.py", "vector_extract.py", "vs1_codec.py"):
            adb("push", f"{wt}/{pkg}/{f}", f"{REMOTE}/DC/{pkg}/")
    elif a[0] == "put":
        for f in a[1:]:
            adb("push", f"{HERE}/{f}", f"{REMOTE}/{f}")
    elif a[0] == "pull":
        adb("pull", f"{REMOTE}/{a[1]}", f"{HERE}/{a[1]}")
    elif a[0] == "clean":
        adb("shell", f'echo fio | sudo -S -p "" rm -rf {REMOTE}; ls -d {REMOTE} 2>&1 || true', check=False)


if __name__ == "__main__":
    main()
