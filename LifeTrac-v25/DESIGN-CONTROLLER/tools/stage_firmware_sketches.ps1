# stage_firmware_sketches.ps1
#
# Round 15 (2026-04-28): mirror the "Stage shared sources into sketch" steps
# of .github/workflows/arduino-ci.yml so a developer can run the same
# arduino-cli compiles locally that the workflow runs in CI.
#
# Why staging exists: each sketch's .ino and .cpp files include shared headers
# from firmware/common/lora_proto/ and firmware/common/shared_mem.h. Arduino's
# build temp dir cannot resolve relative paths like "../common/...", so we
# vendor those sources into a per-sketch ./src/ subfolder (which Arduino IDE
# auto-compiles via its src/** convention) and rewrite the .ino includes to
# point at "src/lora_proto/lora_proto.h" instead of "../common/lora_proto/...".
#
# The src/ trees are .gitignored. firmware/common/ remains the canonical
# source — never edit the staged copies.

[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot   # .../DESIGN-CONTROLLER
$fw   = Join-Path $root 'firmware'
$common = Join-Path $fw 'common'

function Stage-Sketch {
    param(
        [Parameter(Mandatory=$true)] [string] $SketchDir,
        [string[]] $LoraProto = @(),
        [string[]] $ExtraFiles = @()
    )
    $src = Join-Path $SketchDir 'src'
    if (Test-Path $src) { Remove-Item -Recurse -Force $src }
    New-Item -ItemType Directory -Force -Path $src | Out-Null
    if ($LoraProto) {
        Copy-Item -Recurse -Force (Join-Path $common 'lora_proto') $src
    }
    foreach ($f in $ExtraFiles) {
        Copy-Item -Force (Join-Path $common $f) $src
    }
    Write-Host "Staged $SketchDir/src ($($ExtraFiles.Count + ($LoraProto.Count -gt 0)) source group(s))"
}

Stage-Sketch -SketchDir (Join-Path $fw 'handheld_mkr')   -LoraProto @('lora_proto') -ExtraFiles @('lifetrac_build_config.h')
Stage-Sketch -SketchDir (Join-Path $fw 'tractor_h7')     -LoraProto @('lora_proto') -ExtraFiles @('shared_mem.h', 'lifetrac_build_config.h')
# Round 17 (2026-04-28): tractor_h7_m4 is its own sketch folder for the
# Portenta H7 M4 co-processor. Only shared_mem.h is shared with the M7.
Stage-Sketch -SketchDir (Join-Path $fw 'tractor_h7_m4')  -LoraProto @() -ExtraFiles @('shared_mem.h')

Write-Host ""
Write-Host "Done. Now compile with:"
Write-Host '  $flags="-DLIFETRAC_ALLOW_UNCONFIGURED_KEY -DLIFETRAC_ALLOW_STUB_CRYPTO"'
Write-Host '  arduino-cli compile --fqbn arduino:samd:mkrwan1310 `'
Write-Host '    --build-property "compiler.cpp.extra_flags=$flags" `'
Write-Host '    --build-property "compiler.c.extra_flags=$flags" `'
Write-Host "    $fw\handheld_mkr"
Write-Host ""
Write-Host "M4 watchdog (Round 17 - its own sketch + FQBN menu option):"
Write-Host '  arduino-cli compile --fqbn "arduino:mbed_portenta:envie_m7:target_core=cm4" `'
Write-Host "    $fw\tractor_h7_m4"
Write-Host ""
Write-Host "IMPORTANT: use compiler.cpp.extra_flags / compiler.c.extra_flags, not"
Write-Host "build.extra_flags. The latter clobbers the board's own flags including"
Write-Host "-DUSE_BQ24195L_PMIC, which gates LORA_IRQ on MKR WAN 1310."
