$ErrorActionPreference = 'Stop'
# External tool stderr should be captured into the transcript, not treated as
# a terminating PowerShell error.
$PSNativeCommandUseErrorActionPreference = $false

$repo = 'c:/Users/dorkm/Documents/GitHub/LifeTrac'
$cli = 'c:/Users/dorkm/AppData/Local/Programs/ArduinoCLI/arduino-cli.exe'
$sketch = "$repo/LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7"
$logDir = "$repo/LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05"
$log = "$logDir/compile_preflight.log"
$buildRoot = "$env:TEMP/lifetrac_gate1_preflight"

New-Item -ItemType Directory -Force -Path $logDir | Out-Null
Set-Location $repo
New-Item -ItemType Directory -Force -Path "$sketch/src" | Out-Null
Copy-Item -Recurse -Force "$repo/LifeTrac-v25/DESIGN-CONTROLLER/firmware/common/lora_proto" "$sketch/src/"
Copy-Item -Force "$repo/LifeTrac-v25/DESIGN-CONTROLLER/firmware/common/shared_mem.h" "$sketch/src/"
Copy-Item -Recurse -Force "$sketch/murata_host" "$sketch/src/"

$header = @()
$header += '=== Gate 1 Compile Preflight (X8 FQBN) ==='
$header += ('Date: ' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss K'))
$header += ('CLI: ' + $cli)
$header += ('Sketch: ' + $sketch)
$header += ''
$header += 'CLI Version:'
$header += (& $cli version)
$header += ''
$header += 'Installed cores:'
$header += (& $cli core list)
$header += ''
$header += 'Relevant boards:'
$header += (& $cli board listall | Select-String 'portenta_x8|envie_m7' | ForEach-Object { $_.Line })
$header += ''
Set-Content -Path $log -Encoding utf8 -Value $header

function Run-CompileCase {
    param(
        [string]$Name,
        [string]$Flags
    )

    $tmpOut = Join-Path $env:TEMP ("copilot_ard_compile_" + [guid]::NewGuid().ToString() + ".out.log")
    $tmpErr = Join-Path $env:TEMP ("copilot_ard_compile_" + [guid]::NewGuid().ToString() + ".err.log")
    $caseBuild = Join-Path $buildRoot $Name
    Remove-Item -Recurse -Force $caseBuild -ErrorAction SilentlyContinue
    $arg = ('compile --clean --jobs 1 --build-path "{0}" --fqbn arduino:mbed_portenta:portenta_x8 --build-property "compiler.cpp.extra_flags={1}" --build-property "compiler.c.extra_flags={1}" "{2}"' -f $caseBuild, $Flags, $sketch)

    Add-Content -Path $log -Value ("=== Case: " + $Name + " ===")
    Add-Content -Path $log -Value ("Args: " + $arg)

    $proc = Start-Process -FilePath $cli -ArgumentList $arg -NoNewWindow -Wait -PassThru -RedirectStandardOutput $tmpOut -RedirectStandardError $tmpErr
    $exitCode = $proc.ExitCode

    Add-Content -Path $log -Value ("ExitCode: " + $exitCode)
    if (Test-Path $tmpOut) {
        Add-Content -Path $log -Value '--- stdout ---'
        Add-Content -Path $log -Value (Get-Content -Path $tmpOut -Raw)
    }
    if (Test-Path $tmpErr) {
        Add-Content -Path $log -Value '--- stderr ---'
        Add-Content -Path $log -Value (Get-Content -Path $tmpErr -Raw)
    }
    Remove-Item -Force $tmpOut,$tmpErr -ErrorAction SilentlyContinue
    Add-Content -Path $log -Value ''

    return $exitCode
}

$common = '-DLIFETRAC_ALLOW_UNCONFIGURED_KEY -DLIFETRAC_ALLOW_STUB_CRYPTO'
$methodG = $common + ' -DLIFETRAC_USE_METHOD_G_HOST=1 -DLIFETRAC_MH_SERIAL=Serial1'
$methodGBench = $methodG

$ecDefault = Run-CompileCase -Name 'x8-default' -Flags $common
$ecMethodG = Run-CompileCase -Name 'x8-methodg-serial1' -Flags $methodG
$ecMethodGBench = Run-CompileCase -Name 'x8-methodg-serial1-benchlog-built-in' -Flags $methodGBench

$summary = "gate1-summary default=$ecDefault methodg=$ecMethodG benchlog=$ecMethodGBench"
Add-Content -Path $log -Value ('Summary: ' + $summary)
Write-Output $summary
if (($ecDefault -eq 0) -and ($ecMethodG -eq 0) -and ($ecMethodGBench -eq 0)) {
    Write-Output 'Gate1 compile matrix PASS'
    exit 0
}

Write-Output 'Gate1 compile matrix FAIL'
exit 1
