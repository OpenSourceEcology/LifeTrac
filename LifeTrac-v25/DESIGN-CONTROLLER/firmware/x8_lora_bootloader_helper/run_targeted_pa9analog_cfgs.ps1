param(
    [string]$AdbSerial = "2E2C1209DABC240B",
    [string]$RepoRoot  = ""
)
<#
.SYNOPSIS
    Run halted cfgs 70a and 71a (targeted PA9=ANALOG tests) with probe+release sequencing.
  Evidence goes to T6_targeted_70_71_<timestamp>/ under bench-evidence/.
#>

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$ScriptRoot = if ($PSScriptRoot) { $PSScriptRoot } else { (Get-Location).Path }

function Resolve-RepoRoot {
    if ($RepoRoot -and (Test-Path -LiteralPath $RepoRoot)) {
        return (Resolve-Path -LiteralPath $RepoRoot).Path
    }
    return (Resolve-Path (Join-Path $ScriptRoot "../../../")).Path
}

function Invoke-Adb {
    param([string[]]$Args)
    $prefix = if ($AdbSerial) { @("-s", $AdbSerial) } else { @() }
    & adb @prefix @Args
    # Return exit code only (stdout goes to caller's pipeline or is discarded)
}

$null = Get-Command adb -ErrorAction Stop
$repo = Resolve-RepoRoot
$helperDir  = Join-Path $repo "DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper"
$evidRoot   = Join-Path $repo "DESIGN-CONTROLLER/bench-evidence"
$stamp      = Get-Date -Format "yyyy-MM-dd_HHmmss"
$evidDir    = Join-Path $evidRoot "T6_targeted_pa9analog_$stamp"
$null = New-Item -ItemType Directory -Force -Path $evidDir

Write-Host "=== Targeted cfg 70/71 run ==="
Write-Host "Evidence: $evidDir"

# Push helper toolkit (cfgs, scripts, firmware already on board)
Write-Host "`nPushing helper toolkit..."
& adb $(if ($AdbSerial) { "-s"; $AdbSerial }) push "$helperDir/." "/tmp/lifetrac_p0c/" | Out-Null
if ($LASTEXITCODE -ne 0) { throw "adb push failed (rc=$LASTEXITCODE)" }
Write-Host "Push complete."

# Cfgs/scripts are pushed via toolkit sync above.

# Helper: run OpenOCD (halted cfg), probe while H7 is halted, then release H7.
function Run-CfgProbeHalted {
    param(
        [string]$CfgHaltedName,   # e.g. "70a_targeted_pa9_analog_pa11_low_halted"
        [string]$EvidDir,
        [string]$ReleaseCfg = "99_release_and_reset"
    )

    $cfgFile    = "/tmp/lifetrac_p0c/$CfgHaltedName.cfg"
    $relCfgFile = "/tmp/lifetrac_p0c/$ReleaseCfg.cfg"
    $ocdLog     = "/tmp/p0c_targeted_ocd.log"
    $probLog    = "/tmp/p0c_targeted_probe.log"

    Write-Host "`n--- Running halted cfg $CfgHaltedName ---"

    # 1. Run OpenOCD: halt H7, force GPIO states, pulse PF4 reset, KEEP HALTED
    $ocdCmd = "echo fio | sudo -S -p '' openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f $cfgFile"
    Write-Host "  Step 1: OpenOCD halt+force..."
    & adb $(if ($AdbSerial) { "-s"; $AdbSerial }) exec-out "sh -lc `"$ocdCmd > $ocdLog 2>&1`""
    $ocdContent = & adb $(if ($AdbSerial) { "-s"; $AdbSerial }) exec-out "cat $ocdLog 2>/dev/null || echo '(no log)'"
    Write-Host "  OCD log: $ocdContent"

    # 2. Wait for Murata to boot after PF4 reset (~700ms nominal)
    Write-Host "  Step 2: Waiting 800ms for Murata boot..."
    Start-Sleep -Milliseconds 800

    # 3. Run Stage1 probe while H7 is halted (GPIO states locked)
    Write-Host "  Step 3: Running probe (H7 halted, GPIO states locked)..."
    $probeCmd = "echo fio | sudo -S -p '' python3 /tmp/lifetrac_p0c/method_g_stage1_probe.py --dev /dev/ttymxc3 --baud 921600 --boot-timeout 2.0"
    $probeOut = & adb $(if ($AdbSerial) { "-s"; $AdbSerial }) exec-out "sh -lc `"$probeCmd 2>&1 | tee $probLog`""
    Write-Host "  Probe output: $($probeOut | Out-String)"

    # 4. Release H7 (resume firmware)
    Write-Host "  Step 4: Releasing H7..."
    $relCmd = "echo fio | sudo -S -p '' openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f $relCfgFile"
    & adb $(if ($AdbSerial) { "-s"; $AdbSerial }) exec-out "sh -lc `"$relCmd > /tmp/p0c_release_ocd.log 2>&1`""

    # 5. Pull evidence
    $localOcd  = Join-Path $EvidDir "$CfgHaltedName.openocd.log"
    $localProb = Join-Path $EvidDir "$CfgHaltedName.probe.log"
    & adb $(if ($AdbSerial) { "-s"; $AdbSerial }) pull $ocdLog $localOcd | Out-Null
    & adb $(if ($AdbSerial) { "-s"; $AdbSerial }) pull $probLog $localProb | Out-Null

    # Show probe result
    if (Test-Path $localProb) {
        Write-Host "=== Probe result for $CfgHaltedName ==="
        Get-Content $localProb -Raw | Write-Host
    }
}

# Also run the LoRa diagnostic probes first
Write-Host "`n=== Running probe_lora_dt.sh ==="
$dtOut = & adb $(if ($AdbSerial) { "-s"; $AdbSerial }) exec-out "sh -lc `"echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/probe_lora_dt.sh 2>&1`""
$dtOutStr = $dtOut | Out-String
Write-Host $dtOutStr
$dtOutStr | Set-Content -LiteralPath (Join-Path $evidDir "probe_lora_dt.txt")

Write-Host "`n=== Running probe_lora_userspace.sh ==="
$usOut = & adb $(if ($AdbSerial) { "-s"; $AdbSerial }) exec-out "sh -lc `"echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/probe_lora_userspace.sh 2>&1`""
$usOutStr = $usOut | Out-String
Write-Host $usOutStr
$usOutStr | Set-Content -LiteralPath (Join-Path $evidDir "probe_lora_userspace.txt")

# Now run the two targeted cfgs
# Run halted variants (H7 stays halted during probe so GPIO override persists)
Run-CfgProbeHalted -CfgHaltedName "70a_targeted_pa9_analog_pa11_low_halted"  -EvidDir $evidDir
Run-CfgProbeHalted -CfgHaltedName "71a_targeted_pa9_analog_pa11_high_halted" -EvidDir $evidDir

Write-Host "`n=== Done. Evidence in: $evidDir ==="
