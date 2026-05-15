# run_tests.ps1 — run base_station unittest suite with process isolation.
#
# Background (W2-09 follow-up): the repo has two `image_pipeline` packages
# on disk (base-side at base_station/image_pipeline/, X8-side at
# firmware/tractor_x8/image_pipeline/). They share a top-level package
# name, so whichever side imports first wins inside a single Python
# process and the other side fails with ModuleNotFoundError on its own
# submodules (e.g. `image_pipeline.tile_cache`). Until the rename slated
# for IP-W2-10, this script sidesteps the collision by running each test
# *file* in its own `py -3 -m unittest` invocation. Slower than discover,
# but deterministic and lets CI go fully green today.
#
# Usage (from base_station/):
#   .\run_tests.ps1                # run every tests/test_*.py
#   .\run_tests.ps1 -Filter x8     # regex match against file name
#   .\run_tests.ps1 -StopOnFail    # exit on first failing file
#   .\run_tests.ps1 -VerboseTests  # pass -v to each unittest invocation
#
# Exit code: 0 if every file passed, 1 otherwise.

[CmdletBinding()]
param(
    [string]$Filter = "",
    [switch]$StopOnFail,
    [switch]$VerboseTests
)

$ErrorActionPreference = "Stop"

# Locate the tests directory relative to this script so the script works
# regardless of caller's $PWD.
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$TestsDir  = Join-Path $ScriptDir "tests"

if (-not (Test-Path $TestsDir)) {
    Write-Error "tests directory not found at $TestsDir"
    exit 2
}

# lora_bridge import refuses to load without a configured fleet key; the
# whole base-station test suite assumes this env knob is set.
$env:LIFETRAC_ALLOW_UNCONFIGURED_KEY = "1"

# Several *_sil tests reuse helpers from sibling test files
# (`from test_axis_ramp_sil import ...`). Under `unittest discover`
# the tests/ dir lands on sys.path automatically, but `python -m
# unittest tests.<mod>` only adds the current dir. Prepend tests/ to
# PYTHONPATH so every subprocess we spawn resolves those siblings.
$prevPyPath = $env:PYTHONPATH
$env:PYTHONPATH = if ($prevPyPath) { "$TestsDir;$prevPyPath" } else { $TestsDir }

# Force base_station/ on sys.path so `import lora_proto` etc. resolves
# the same way `py -3 -m unittest tests.<x>` does when launched from this
# directory. Push-Location keeps the caller's cwd untouched on exit.
Push-Location $ScriptDir
try {
    $files = Get-ChildItem -Path $TestsDir -Filter "test_*.py" |
        Where-Object { -not $Filter -or $_.Name -match $Filter } |
        Sort-Object Name

    $totalFiles  = $files.Count
    $passedFiles = 0
    $failedFiles = @()

    Write-Host ""
    Write-Host "[run_tests] $totalFiles test file(s) under $TestsDir"
    if ($Filter) { Write-Host "[run_tests] filter: *$Filter*" }
    Write-Host ""

    foreach ($f in $files) {
        $mod = "tests." + [System.IO.Path]::GetFileNameWithoutExtension($f.Name)
        $verb = if ($VerboseTests) { "-v" } else { "" }
        Write-Host -NoNewline "  $($f.Name) ... "

        # Each file in its own python process => sys.modules is fresh,
        # so the X8-side and base-side image_pipeline packages cannot
        # collide. Capture both streams together so a Traceback shows up.
        # Switch ErrorActionPreference to Continue around the native call
        # because PowerShell otherwise treats *any* line on stderr (e.g.
        # the perfectly normal "camera_service: CMD_ENCODE_MODE rejected"
        # log lines emitted during negative tests) as a script-terminating
        # error. We judge pass/fail purely by $LASTEXITCODE.
        $prevEAP = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        $output = & py -3 -m unittest $verb $mod 2>&1
        $exit = $LASTEXITCODE
        $ErrorActionPreference = $prevEAP

        if ($exit -eq 0) {
            Write-Host "ok" -ForegroundColor Green
            $passedFiles++
        } else {
            Write-Host "FAIL (exit=$exit)" -ForegroundColor Red
            # Tail of output is the most useful piece (Traceback + summary).
            $output | Select-Object -Last 30 | ForEach-Object {
                Write-Host "      $_" -ForegroundColor DarkGray
            }
            $failedFiles += $f.Name
            if ($StopOnFail) { break }
        }
    }

    Write-Host ""
    Write-Host "----------------------------------------------------------------------"
    Write-Host "[run_tests] passed: $passedFiles / $totalFiles"
    if ($failedFiles.Count -gt 0) {
        Write-Host "[run_tests] failed files:" -ForegroundColor Red
        $failedFiles | ForEach-Object { Write-Host "    $_" -ForegroundColor Red }
        exit 1
    }
    Write-Host "[run_tests] OK" -ForegroundColor Green
    exit 0
}
finally {
    Pop-Location
    $env:PYTHONPATH = $prevPyPath
}
