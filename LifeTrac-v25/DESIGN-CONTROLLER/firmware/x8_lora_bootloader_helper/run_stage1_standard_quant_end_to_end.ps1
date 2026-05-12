param(
    [string]$AdbSerial = "",
    [string]$RepoRoot = "",
    [string]$LocalImage = "",
    [int]$Cycles = 20,
    [int]$CycleTimeoutSec = 180,
    [bool]$RunGate = $true
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
if (Get-Variable -Name PSNativeCommandUseErrorActionPreference -ErrorAction SilentlyContinue) {
    $PSNativeCommandUseErrorActionPreference = $false
}

function Resolve-RepoRoot {
    param(
        [string]$Hint,
        [string]$ScriptRoot
    )

    if ($Hint -and (Test-Path -LiteralPath $Hint)) {
        return (Resolve-Path -LiteralPath $Hint).Path
    }

    return (Resolve-Path (Join-Path $ScriptRoot "../../../")).Path
}

function Parse-KeyValueFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    $map = @{}
    if (-not (Test-Path -LiteralPath $Path)) {
        return $map
    }

    Get-Content -LiteralPath $Path | ForEach-Object {
        $line = $_.Trim()
        if (-not $line) { return }
        $idx = $line.IndexOf("=")
        if ($idx -lt 1) { return }
        $key = $line.Substring(0, $idx).Trim()
        $value = $line.Substring($idx + 1).Trim()
        $map[$key] = $value
    }

    return $map
}

function Increment-Count {
    param(
        [Parameter(Mandatory = $true)]
        [hashtable]$Counter,
        [Parameter(Mandatory = $true)]
        [string]$Key
    )

    if (-not $Counter.ContainsKey($Key)) {
        $Counter[$Key] = 0
    }
    $Counter[$Key] = [int]$Counter[$Key] + 1
}

function Append-TextWithRetry {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Value,
        [int]$MaxAttempts = 8,
        [int]$DelayMs = 120
    )

    for ($attempt = 1; $attempt -le $MaxAttempts; $attempt++) {
        try {
            Add-Content -LiteralPath $Path -Value $Value
            return
        } catch [System.IO.IOException] {
            if ($attempt -eq $MaxAttempts) {
                throw
            }
            Start-Sleep -Milliseconds $DelayMs
        }
    }
}

function Invoke-Adb {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Args,
        [string]$Serial
    )

    $prefix = @()
    if ($Serial) {
        $prefix = @("-s", $Serial)
    }

    & adb @prefix @Args
    return $LASTEXITCODE
}

function Invoke-AdbExecOutWithTimeout {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Command,
        [string]$Serial,
        [int]$TimeoutSec = 180
    )

    $stdoutPath = [System.IO.Path]::GetTempFileName()
    $stderrPath = [System.IO.Path]::GetTempFileName()

    try {
        $escapedCommand = $Command.Replace('"', '\"')
        $adbArgString = ""
        if ($Serial) {
            $adbArgString += "-s `"$Serial`" "
        }
        $adbArgString += "exec-out `"$escapedCommand`""

        $proc = Start-Process -FilePath "adb" `
                              -ArgumentList $adbArgString `
                              -PassThru `
                              -NoNewWindow `
                              -RedirectStandardOutput $stdoutPath `
                              -RedirectStandardError $stderrPath

        $finished = $proc.WaitForExit($TimeoutSec * 1000)
        if (-not $finished) {
            try { $proc.Kill() } catch {}
            try { $proc.WaitForExit() } catch {}

            $stdoutText = ""
            $stderrText = ""
            if (Test-Path -LiteralPath $stdoutPath) {
                $stdoutText = Get-Content -LiteralPath $stdoutPath -Raw -ErrorAction SilentlyContinue
            }
            if (Test-Path -LiteralPath $stderrPath) {
                $stderrText = Get-Content -LiteralPath $stderrPath -Raw -ErrorAction SilentlyContinue
            }

            return @{
                TimedOut = $true
                LauncherRc = 124
                OutputText = ($stdoutText + $stderrText)
            }
        }

        $stdoutText = ""
        $stderrText = ""
        if (Test-Path -LiteralPath $stdoutPath) {
            $stdoutText = Get-Content -LiteralPath $stdoutPath -Raw -ErrorAction SilentlyContinue
        }
        if (Test-Path -LiteralPath $stderrPath) {
            $stderrText = Get-Content -LiteralPath $stderrPath -Raw -ErrorAction SilentlyContinue
        }

        return @{
            TimedOut = $false
            LauncherRc = $proc.ExitCode
            OutputText = ($stdoutText + $stderrText)
        }
    } finally {
        Remove-Item -LiteralPath $stdoutPath -ErrorAction SilentlyContinue
        Remove-Item -LiteralPath $stderrPath -ErrorAction SilentlyContinue
    }
}

if ($Cycles -lt 1) {
    throw "Cycles must be >= 1"
}

if ($CycleTimeoutSec -lt 10) {
    throw "CycleTimeoutSec must be >= 10"
}

$scriptRoot = if ($PSScriptRoot) { $PSScriptRoot } else { (Get-Location).Path }
$repo = Resolve-RepoRoot -Hint $RepoRoot -ScriptRoot $scriptRoot
$helperDir = Join-Path $repo "DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper"
$benchEvidenceRoot = Join-Path $repo "DESIGN-CONTROLLER/bench-evidence"

$imagePath = $LocalImage
if (-not $imagePath) {
    $imagePath = Join-Path $repo "DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.bin"
}

if (-not (Test-Path -LiteralPath $helperDir)) {
    throw "Helper directory not found: $helperDir"
}

if (-not (Test-Path -LiteralPath $imagePath)) {
    throw "Image not found: $imagePath"
}

$resolvedImage = (Resolve-Path -LiteralPath $imagePath).Path
$remoteImage = "/tmp/lifetrac_p0c/firmware.bin"

$null = Get-Command adb -ErrorAction Stop

Write-Host "Pushing helper toolkit once before quant loop..."
$pushToolkitRc = Invoke-Adb -Args @("push", "$helperDir/.", "/tmp/lifetrac_p0c/") -Serial $AdbSerial
if ($pushToolkitRc -ne 0) {
    throw "adb push toolkit failed with exit code $pushToolkitRc"
}

Write-Host "Pushing image once before quant loop..."
$pushImageRc = Invoke-Adb -Args @("push", $resolvedImage, $remoteImage) -Serial $AdbSerial
if ($pushImageRc -ne 0) {
    throw "adb push image failed with exit code $pushImageRc"
}

$chmodCmd = "chmod +x /tmp/lifetrac_p0c/*.sh"
$chmodPrefix = @()
if ($AdbSerial) { $chmodPrefix = @("-s", $AdbSerial) }
& adb @chmodPrefix exec-out $chmodCmd | Out-Null
$chmodRc = $LASTEXITCODE
if ($chmodRc -ne 0) {
    throw "chmod on remote helper scripts failed with exit code $chmodRc"
}

# Pre-flight: ensure H7 SWD pins (gpio8/10/15) are exported and NRST (gpio10) is held
# high so OpenOCD imx_gpio bitbang can read SWD DPIDR. On freshly-flashed X8 images
# (kernel 6.1.x LMP), gpio10 boots low, holding the H7 in reset and causing
# "Error connecting DP: cannot read IDR". See AI NOTES 2026-05-12 recovery plan.
$gpioPreflight = "for n in 8 10 15; do [ -d /sys/class/gpio/gpio`$n ] || echo `$n > /sys/class/gpio/export 2>/dev/null; done; echo out > /sys/class/gpio/gpio10/direction 2>/dev/null; echo 1 > /sys/class/gpio/gpio10/value 2>/dev/null; cat /sys/class/gpio/gpio10/value"
$gpioPreflightRemote = "echo fio | sudo -S -p '' bash -c `"$gpioPreflight`""
& adb @chmodPrefix exec-out $gpioPreflightRemote | Out-Null
Write-Host "GPIO preflight complete (gpio10 driven high to release H7 NRST)."

$stamp = Get-Date -Format "yyyy-MM-dd_HHmmss_fff"
$stamp = "$stamp-$PID"
$quantDir = Join-Path $benchEvidenceRoot "T6_stage1_standard_quant_$stamp"
$null = New-Item -ItemType Directory -Force -Path $quantDir

$resultsCsv  = Join-Path $quantDir "results.csv"
$summaryTxt  = Join-Path $quantDir "summary.txt"
$launcherLog = Join-Path $quantDir "launcher.log"
$statusTxt   = Join-Path $quantDir "status.txt"

"cycle,launcher_rc,evidence_dir,final_result,elapsed_s,sync_ok,getid_ok,erase_ok,write_ok,verify_ok,boot_ok" | Set-Content -LiteralPath $resultsCsv

$resultCounts = @{}
$launcherFailCount = 0
$timeoutCount = 0
$cycleStart = (Get-Date).ToUniversalTime().ToString("o")

# Write initial status so incomplete/interrupted runs are identifiable
@(
    "STATUS=RUNNING",
    "CYCLES_PLANNED=$Cycles",
    "CYCLES_COMPLETED=0",
    "START_UTC=$cycleStart"
) | Set-Content -LiteralPath $statusTxt

for ($cycle = 1; $cycle -le $Cycles; $cycle++) {
    $cycleHeader = "=== cycle $cycle/$Cycles $(Get-Date -Format o) ==="
    Append-TextWithRetry -Path $launcherLog -Value $cycleHeader
    Write-Host $cycleHeader

    $serialForRemote = if ($AdbSerial) { $AdbSerial } else { "(auto)" }
    $runnerCmd = "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_stage1_standard_contract.sh $remoteImage '$serialForRemote'"
    $remoteWrappedCmd = "sh -lc '$runnerCmd; rc=`$?; printf ""__STD_RC__=%s\n"" ""`$rc""'"

    $invokeResult = Invoke-AdbExecOutWithTimeout -Command $remoteWrappedCmd -Serial $AdbSerial -TimeoutSec $CycleTimeoutSec
    $launcherRc = [int]$invokeResult.LauncherRc
    $outputText = [string]$invokeResult.OutputText
    if ($invokeResult.TimedOut) {
        $timeoutCount++
        $timeoutMsg = "WARN: cycle $cycle timed out after ${CycleTimeoutSec}s (adb exec-out killed)"
        Append-TextWithRetry -Path $launcherLog -Value $timeoutMsg
        Write-Host $timeoutMsg
    }
    Append-TextWithRetry -Path $launcherLog -Value $outputText

    $evidenceDir = ""
    $runDirMatch = [regex]::Match($outputText, "RUN_OUTDIR=([^\r\n]+)")
    if ($runDirMatch.Success) {
        $remoteRunDir = $runDirMatch.Groups[1].Value.Trim()
        $pullRc = Invoke-Adb -Args @("pull", $remoteRunDir, $benchEvidenceRoot) -Serial $AdbSerial
        if ($pullRc -eq 0) {
            $runLeaf = Split-Path -Leaf $remoteRunDir
            $evidenceDir = Join-Path $benchEvidenceRoot $runLeaf
        } else {
            Append-TextWithRetry -Path $launcherLog -Value "WARN: adb pull failed for $remoteRunDir rc=$pullRc"
        }
    }

    $finalResult = if ($invokeResult.TimedOut) { "RUNNER_TIMEOUT" } else { "RUNNER_FAIL" }
    $elapsed = ""
    $syncOk = ""
    $getIdOk = ""
    $eraseOk = ""
    $writeOk = ""
    $verifyOk = ""
    $bootOk = ""

    if ($evidenceDir -and (Test-Path -LiteralPath $evidenceDir)) {
        $summaryPath = Join-Path $evidenceDir "summary.txt"
        $kv = Parse-KeyValueFile -Path $summaryPath
        if ($kv.ContainsKey("FINAL_RESULT")) {
            $finalResult = $kv["FINAL_RESULT"]
        }
        if ($kv.ContainsKey("ELAPSED_S")) { $elapsed = $kv["ELAPSED_S"] }
        if ($kv.ContainsKey("SYNC_OK")) { $syncOk = $kv["SYNC_OK"] }
        if ($kv.ContainsKey("GETID_OK")) { $getIdOk = $kv["GETID_OK"] }
        if ($kv.ContainsKey("ERASE_OK")) { $eraseOk = $kv["ERASE_OK"] }
        if ($kv.ContainsKey("WRITE_OK")) { $writeOk = $kv["WRITE_OK"] }
        if ($kv.ContainsKey("VERIFY_OK")) { $verifyOk = $kv["VERIFY_OK"] }
        if ($kv.ContainsKey("BOOT_OK")) { $bootOk = $kv["BOOT_OK"] }
    }

    $stdRc = $launcherRc
    $stdRcMatch = [regex]::Match($outputText, "__STD_RC__=(\d+)")
    if ($stdRcMatch.Success) {
        $stdRc = [int]$stdRcMatch.Groups[1].Value
    }

    if (($launcherRc -ne 0) -or ($stdRc -ne 0)) {
        $launcherFailCount++
    }

    Increment-Count -Counter $resultCounts -Key $finalResult

    $safeEvidence = $evidenceDir -replace ',', ';'
    Append-TextWithRetry -Path $resultsCsv -Value "$cycle,$stdRc,$safeEvidence,$finalResult,$elapsed,$syncOk,$getIdOk,$eraseOk,$writeOk,$verifyOk,$bootOk"
    Write-Host "cycle=$cycle launcher_rc=$launcherRc std_rc=$stdRc final_result=$finalResult"

    # Update live status after each cycle
    @(
        "STATUS=RUNNING",
        "CYCLES_PLANNED=$Cycles",
        "CYCLES_COMPLETED=$cycle",
        "LAST_CYCLE_UTC=$(Get-Date -Format o)",
        "LAST_CYCLE_RESULT=$finalResult",
        "START_UTC=$cycleStart"
    ) | Set-Content -LiteralPath $statusTxt
}

$cycleEnd = (Get-Date).ToUniversalTime().ToString("o")

# Mark run complete before writing summary
@(
    "STATUS=COMPLETE",
    "CYCLES_PLANNED=$Cycles",
    "CYCLES_COMPLETED=$Cycles",
    "START_UTC=$cycleStart",
    "END_UTC=$cycleEnd"
) | Set-Content -LiteralPath $statusTxt

$orderedResults = $resultCounts.GetEnumerator() | Sort-Object Name
$resultLines = @()
foreach ($entry in $orderedResults) {
    $resultLines += ("FINAL_RESULT_{0}={1}" -f $entry.Key, $entry.Value)
}

$summary = @(
    "RUN_ID=T6_stage1_standard_quant_$stamp",
    "CYCLES=$Cycles",
    "CYCLE_TIMEOUT_SEC=$CycleTimeoutSec",
    "BOARD_SERIAL=$AdbSerial",
    "CYCLE_START_UTC=$cycleStart",
    "CYCLE_END_UTC=$cycleEnd",
    "LAUNCHER_FAIL_COUNT=$launcherFailCount",
    "TIMEOUT_COUNT=$timeoutCount",
    "RESULTS_CSV=$resultsCsv",
    "LAUNCHER_LOG=$launcherLog"
) + $resultLines

$summary | Set-Content -LiteralPath $summaryTxt
Get-Content -LiteralPath $summaryTxt | Write-Host

if ($RunGate) {
    $gateScript = Join-Path $helperDir "run_stage1_quant_gate.ps1"
    if (-not (Test-Path -LiteralPath $gateScript)) {
        throw "Quant gate script not found: $gateScript"
    }

    Write-Host "Running quant gate check..."
    & $gateScript -SummaryPath $summaryTxt -ExpectedCycles $Cycles
    $gateRc = $LASTEXITCODE
    if ($gateRc -ne 0) {
        throw "Quant gate failed with exit code $gateRc"
    }
}
