param(
    [string]$ImageRoot = "$HOME\Downloads\portenta-x8-reflash\934",
    [switch]$RunFlash
)

$ErrorActionPreference = "Stop"

function Assert-PathExists {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Label
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Missing $Label at: $Path"
    }
}

$imageRootResolved = (Resolve-Path -LiteralPath $ImageRoot).Path
$mfgRoot = Join-Path $imageRootResolved "mfgtool-files-portenta-x8"
$uuuPath = Join-Path $imageRootResolved "..\uuu.exe"
if (-not (Test-Path -LiteralPath $uuuPath)) {
    $uuuPath = Join-Path (Split-Path -Parent $imageRootResolved) "uuu.exe"
}

Assert-PathExists -Path $imageRootResolved -Label "image root"
Assert-PathExists -Path (Join-Path $imageRootResolved "imx-boot-portenta-x8") -Label "imx boot image"
Assert-PathExists -Path (Join-Path $imageRootResolved "u-boot-portenta-x8.itb") -Label "u-boot image"
Assert-PathExists -Path (Join-Path $imageRootResolved "sit-portenta-x8.bin") -Label "SIT image"
Assert-PathExists -Path (Join-Path $imageRootResolved "lmp-factory-image-portenta-x8.wic") -Label "factory WIC image"
Assert-PathExists -Path $mfgRoot -Label "mfgtool directory"
Assert-PathExists -Path (Join-Path $mfgRoot "full_image.uuu") -Label "uuu script"
Assert-PathExists -Path $uuuPath -Label "uuu.exe"

Write-Host "Portenta X8 official recovery bundle is staged and validated."
Write-Host "Image root: $imageRootResolved"
Write-Host "uuu tool : $uuuPath"
Write-Host "uuu script: $(Join-Path $mfgRoot 'full_image.uuu')"

if (-not $RunFlash) {
    Write-Host ""
    Write-Host "Ready to flash, but not running because -RunFlash was not provided."
    Write-Host "Physical steps before running:"
    Write-Host "  1. Put the Portenta X8 Max Carrier BOOT SEL and BOOT DIP switches to ON."
    Write-Host "  2. Use a USB-C to USB-A cable to the target board."
    Write-Host "  3. Then rerun this script with -RunFlash."
    exit 0
}

Push-Location $mfgRoot
try {
    & $uuuPath "full_image.uuu"
    if ($LASTEXITCODE -ne 0) {
        throw "uuu exited with code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}

Write-Host ""
Write-Host "Flashing completed. Set Max Carrier BOOT SEL and BOOT DIP switches back to OFF, then power-cycle the board."