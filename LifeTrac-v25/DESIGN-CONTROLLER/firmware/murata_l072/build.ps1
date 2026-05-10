# Build script (PowerShell) for the main Method G murata_l072 firmware.
# Uses Arduino-bundled arm-none-eabi-gcc and a Windows make implementation.

$ErrorActionPreference = "Stop"

function Resolve-Tool {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Candidates,
        [Parameter(Mandatory = $true)]
        [string]$Label
    )

    foreach ($candidate in $Candidates) {
        if (-not [string]::IsNullOrWhiteSpace($candidate) -and (Test-Path -LiteralPath $candidate)) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    throw "$Label not found. Checked:`n - $($Candidates -join "`n - ")"
}

$scriptRoot = if ($PSScriptRoot) { $PSScriptRoot } else { Split-Path -Parent $PSCommandPath }
$toolchainRoot = Join-Path $env:LOCALAPPDATA "Arduino15\packages\arduino\tools\arm-none-eabi-gcc\7-2017q4\bin"

$makeCandidates = @()
foreach ($candidate in @(
    (Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin\mingw32-make.exe"),
    (Join-Path $env:ProgramFiles "Git\usr\bin\make.exe"),
    (Get-Command mingw32-make -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -ErrorAction SilentlyContinue),
    (Get-Command make -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -ErrorAction SilentlyContinue)
)) {
    if ($candidate -is [string] -and -not [string]::IsNullOrWhiteSpace($candidate)) {
        $makeCandidates += $candidate
    }
}

$gccCandidates = @()
foreach ($candidate in @(
    (Join-Path $toolchainRoot "arm-none-eabi-gcc.exe"),
    (Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -ErrorAction SilentlyContinue)
)) {
    if ($candidate -is [string] -and -not [string]::IsNullOrWhiteSpace($candidate)) {
        $gccCandidates += $candidate
    }
}

$makeExe = Resolve-Tool -Label "make" -Candidates $makeCandidates
$gccExe = Resolve-Tool -Label "arm-none-eabi-gcc" -Candidates $gccCandidates

$crossPrefix = [System.IO.Path]::Combine((Split-Path -Parent $gccExe), "arm-none-eabi-") -replace "\\","/"

Write-Host "Using make: $makeExe"
Write-Host "Using gcc : $gccExe"
Write-Host "Source dir : $scriptRoot"

Push-Location $scriptRoot
try {
    & $makeExe clean
    if ($LASTEXITCODE -ne 0) { throw "make clean failed" }

    & $makeExe CROSS=$crossPrefix all
    if ($LASTEXITCODE -ne 0) { throw "make all failed" }
}
finally {
    Pop-Location
}

$bin = Join-Path $scriptRoot "build\firmware.bin"
if (-not (Test-Path -LiteralPath $bin)) {
    throw "Expected output not found: $bin"
}

$binSize = (Get-Item -LiteralPath $bin).Length
Write-Host "Built firmware.bin = $binSize bytes"