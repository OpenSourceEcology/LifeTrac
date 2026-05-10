param(
    [string]$AdbSerial = "",
    [string]$RepoRoot = "",
    [string]$DelaysMs = "2,5,10",
    [int]$BurstsPerDelay = 20,
    [int]$AttemptsPerBurst = 1,
    [int]$BurstPassMinAck = 1,
    [int]$OpenOcdLifetimeS = 75,
    [string]$RemoteOutRoot = "/tmp/lifetrac_p0c/rom_baseline_burst",
    [int]$PollSec = 20,
    [int]$MaxAdbRecovery = 6,
    [int]$MaxMonitorMinutes = 120
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

$ScriptRoot = if ($PSScriptRoot) {
    $PSScriptRoot
} elseif ($PSCommandPath) {
    Split-Path -Parent $PSCommandPath
} else {
    (Get-Location).Path
}

function Resolve-RepoRoot {
    param([string]$Requested)

    if ($Requested -and (Test-Path -LiteralPath $Requested)) {
        return (Resolve-Path -LiteralPath $Requested).Path
    }

    $candidate = Resolve-Path (Join-Path $ScriptRoot "../../../")
    return $candidate.Path
}

function Get-AdbDeviceTable {
    $output = (& adb devices | Out-String)
    $rows = @()
    foreach ($line in ($output -split "`r?`n")) {
        if (-not $line) { continue }
        if ($line -match "^List of devices attached") { continue }
        if ($line -match "^\* daemon") { continue }
        if ($line -match "^\s*$") { continue }

        $parts = ($line -split "\s+") | Where-Object { $_ -ne "" }
        if ($parts.Count -ge 2) {
            $rows += [pscustomobject]@{
                Serial = $parts[0]
                State  = $parts[1]
            }
        }
    }

    return $rows
}

function Resolve-TargetSerial {
    param([string]$Requested)

    $devices = Get-AdbDeviceTable
    if ($Requested) {
        return $Requested
    }

    $active = $devices | Where-Object { $_.State -eq "device" } | Select-Object -First 1
    if ($null -eq $active) {
        throw "No active ADB device detected. Connect board and retry."
    }

    return $active.Serial
}

function Invoke-AdbRaw {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Serial,
        [Parameter(Mandatory = $true)]
        [string[]]$AdbArgs
    )

    $allArgs = @("-s", $Serial) + $AdbArgs
    $oldEa = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        $out = & adb @allArgs 2>&1
        $rc = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $oldEa
    }

    return [pscustomobject]@{
        ExitCode = $rc
        Output   = ($out | Out-String).Trim()
    }
}

function Test-DeviceOnline {
    param([string]$Serial)

    $devices = Get-AdbDeviceTable
    $row = $devices | Where-Object { $_.Serial -eq $Serial } | Select-Object -First 1
    if ($null -eq $row) { return $false }
    return ($row.State -eq "device")
}

function Recover-AdbConnection {
    param(
        [string]$Serial,
        [int]$MaxAttempts
    )

    for ($i = 1; $i -le $MaxAttempts; $i++) {
        Write-Host "[adb-recover] attempt $i/$MaxAttempts"
        & adb kill-server | Out-Null
        & adb start-server | Out-Null
        & adb reconnect | Out-Null
        Start-Sleep -Seconds 2

        if (Test-DeviceOnline -Serial $Serial) {
            Write-Host "[adb-recover] device restored: $Serial"
            return $true
        }
    }

    return $false
}

function Invoke-AdbWithRecovery {
    param(
        [string]$Serial,
        [string[]]$AdbArgs,
        [int]$MaxRecoveryAttempts,
        [switch]$AllowFailure
    )

    $result = Invoke-AdbRaw -Serial $Serial -AdbArgs $AdbArgs
    if ($result.ExitCode -eq 0) {
        return $result
    }

    if (-not (Recover-AdbConnection -Serial $Serial -MaxAttempts $MaxRecoveryAttempts)) {
        if ($AllowFailure) { return $result }
        throw "ADB command failed and recovery did not restore device.`nArgs: $($AdbArgs -join ' ')`nOutput: $($result.Output)"
    }

    $retry = Invoke-AdbRaw -Serial $Serial -AdbArgs $AdbArgs
    if ($retry.ExitCode -ne 0 -and -not $AllowFailure) {
        throw "ADB command failed after recovery.`nArgs: $($AdbArgs -join ' ')`nOutput: $($retry.Output)"
    }

    return $retry
}

$null = Get-Command adb -ErrorAction Stop

$repo = Resolve-RepoRoot -Requested $RepoRoot
$helperDir = Join-Path $repo "DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper"
$evidenceRoot = Join-Path $repo "DESIGN-CONTROLLER/bench-evidence"
$localLogRoot = Join-Path $evidenceRoot "adb_hardened_logs"
New-Item -ItemType Directory -Force -Path $localLogRoot | Out-Null

$timestamp = Get-Date -Format "yyyy-MM-dd_HHmmss"
$localLog = Join-Path $localLogRoot "rom_matrix_hardened_$timestamp.log"

function Write-LocalLog {
    param([string]$Line)
    $ts = Get-Date -Format "HH:mm:ss"
    $msg = "[$ts] $Line"
    Add-Content -LiteralPath $localLog -Value $msg
    Write-Host $msg
}

$targetSerial = Resolve-TargetSerial -Requested $AdbSerial
Write-LocalLog "target_serial=$targetSerial"

if (-not (Test-Path -LiteralPath $helperDir)) {
    throw "Helper directory not found: $helperDir"
}

Write-LocalLog "pushing_helper_dir"
$push = Invoke-AdbWithRecovery -Serial $targetSerial -AdbArgs @("push", "$helperDir/.", "/tmp/lifetrac_p0c/") -MaxRecoveryAttempts $MaxAdbRecovery
Write-LocalLog ("push_rc={0}" -f $push.ExitCode)

Write-LocalLog "chmod_remote_scripts"
$chmodCmd = "printf 'fio\n' | sudo -S -p '' sh -lc 'chmod +x /tmp/lifetrac_p0c/*.sh'"
$chmod = Invoke-AdbWithRecovery -Serial $targetSerial -AdbArgs @("exec-out", $chmodCmd) -MaxRecoveryAttempts $MaxAdbRecovery
Write-LocalLog ("chmod_rc={0}" -f $chmod.ExitCode)

$remoteDiag = "$RemoteOutRoot/diag_2_5_10_hardened_$timestamp.log"
$launchCmd = "printf 'fio\n' | sudo -S -p '' sh -lc 'mkdir -p $RemoteOutRoot; nohup env DELAYS_MS=$DelaysMs BURSTS_PER_DELAY=$BurstsPerDelay ATTEMPTS_PER_BURST=$AttemptsPerBurst BURST_PASS_MIN_ACK=$BurstPassMinAck OPENOCD_LIFETIME_S=$OpenOcdLifetimeS bash /tmp/lifetrac_p0c/run_rom_baseline_burst_matrix.sh > $remoteDiag 2>&1 < /dev/null & echo STARTED'"

Write-LocalLog "launching_remote_run"
$launch = Invoke-AdbWithRecovery -Serial $targetSerial -AdbArgs @("shell", $launchCmd) -MaxRecoveryAttempts $MaxAdbRecovery
Write-LocalLog ("launch_output={0}" -f $launch.Output)

Start-Sleep -Seconds 2
$latestDirCmd = "ls -1dt $RemoteOutRoot/T6_rom_baseline_burst_* 2>/dev/null | head -1"
$runDir = ""
for ($i = 1; $i -le 10; $i++) {
    $runDirResp = Invoke-AdbWithRecovery -Serial $targetSerial -AdbArgs @("exec-out", "sh -lc '$latestDirCmd'") -MaxRecoveryAttempts $MaxAdbRecovery
    $candidate = ($runDirResp.Output -split "`r?`n" | Select-Object -First 1).Trim()
    if ($candidate) {
        $runDir = $candidate
        break
    }
    Start-Sleep -Seconds 2
}
if (-not $runDir) {
    throw "Could not determine remote run directory."
}

Write-LocalLog "run_dir=$runDir"

$deadline = (Get-Date).AddMinutes($MaxMonitorMinutes)
$ready = $false
while ((Get-Date) -lt $deadline) {
    $statusCmd = "if [ -f $runDir/summary.txt ]; then echo READY; cat $runDir/summary.txt; echo ---DELAY---; cat $runDir/delay_summary.csv; else echo PENDING; tail -n 6 $runDir/run.log 2>/dev/null || true; fi"
    $status = Invoke-AdbWithRecovery -Serial $targetSerial -AdbArgs @("exec-out", "sh -lc '$statusCmd'") -MaxRecoveryAttempts $MaxAdbRecovery
    $text = $status.Output

    if ($text -match "(?m)^READY$") {
        $ready = $true
        Write-LocalLog "remote_run_complete"
        Add-Content -LiteralPath $localLog -Value $text
        break
    }

    $tailLine = ($text -split "`r?`n" | Select-Object -Last 1)
    if (-not $tailLine) { $tailLine = "pending" }
    Write-LocalLog ("monitor_pending={0}" -f $tailLine)
    Start-Sleep -Seconds $PollSec
}

if (-not $ready) {
    throw "Timed out waiting for summary.txt in $runDir"
}

Write-LocalLog "pulling_remote_artifacts"
$pull = Invoke-AdbWithRecovery -Serial $targetSerial -AdbArgs @("pull", $runDir, $evidenceRoot) -MaxRecoveryAttempts $MaxAdbRecovery
Write-LocalLog ("pull_rc={0}" -f $pull.ExitCode)

$localRunDirName = Split-Path -Leaf $runDir
$localRunDir = Join-Path $evidenceRoot $localRunDirName

Write-LocalLog "local_run_dir=$localRunDir"
Write-Output "RUN_DIR=$runDir"
Write-Output "LOCAL_RUN_DIR=$localRunDir"
Write-Output "LOCAL_LOG=$localLog"