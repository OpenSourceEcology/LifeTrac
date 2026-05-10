param(
    [string]$AdbSerial = "",
    [string]$RepoRoot = "",
    [string]$LocalImage = "",
    [int]$Cycles = 20
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

if ($Cycles -lt 1) {
    throw "Cycles must be >= 1"
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

$chmodCmd = "echo fio | sudo -S -p '' bash -lc 'chmod +x /tmp/lifetrac_p0c/*.sh'"
$chmodRc = Invoke-Adb -Args @("exec-out", $chmodCmd) -Serial $AdbSerial
if ($chmodRc -ne 0) {
    throw "chmod on remote helper scripts failed with exit code $chmodRc"
}

$stamp = Get-Date -Format "yyyy-MM-dd_HHmmss_fff"
$stamp = "$stamp-$PID"
$quantDir = Join-Path $benchEvidenceRoot "T6_stage1_standard_quant_$stamp"
$null = New-Item -ItemType Directory -Force -Path $quantDir

$resultsCsv = Join-Path $quantDir "results.csv"
$summaryTxt = Join-Path $quantDir "summary.txt"
$launcherLog = Join-Path $quantDir "launcher.log"

"cycle,launcher_rc,evidence_dir,final_result,elapsed_s,sync_ok,getid_ok,erase_ok,write_ok,verify_ok,boot_ok" | Set-Content -LiteralPath $resultsCsv

$resultCounts = @{}
$launcherFailCount = 0
$cycleStart = (Get-Date).ToUniversalTime().ToString("o")

for ($cycle = 1; $cycle -le $Cycles; $cycle++) {
    $cycleHeader = "=== cycle $cycle/$Cycles $(Get-Date -Format o) ==="
    Append-TextWithRetry -Path $launcherLog -Value $cycleHeader
    Write-Host $cycleHeader

    $serialForRemote = if ($AdbSerial) { $AdbSerial } else { "(auto)" }
    $runnerCmd = "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_stage1_standard_contract.sh $remoteImage '$serialForRemote'"
    $remoteWrappedCmd = "sh -lc '$runnerCmd; rc=`$?; printf ""__STD_RC__=%s\n"" ""`$rc""'"

    $prevErr = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $prefix = @()
    if ($AdbSerial) {
        $prefix = @("-s", $AdbSerial)
    }
    $output = & adb @prefix exec-out $remoteWrappedCmd 2>&1
    $ErrorActionPreference = $prevErr
    $launcherRc = $LASTEXITCODE
    $outputText = ($output | Out-String)
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

    $finalResult = "RUNNER_FAIL"
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
}

$cycleEnd = (Get-Date).ToUniversalTime().ToString("o")

$orderedResults = $resultCounts.GetEnumerator() | Sort-Object Name
$resultLines = @()
foreach ($entry in $orderedResults) {
    $resultLines += ("FINAL_RESULT_{0}={1}" -f $entry.Key, $entry.Value)
}

$summary = @(
    "RUN_ID=T6_stage1_standard_quant_$stamp",
    "CYCLES=$Cycles",
    "BOARD_SERIAL=$AdbSerial",
    "CYCLE_START_UTC=$cycleStart",
    "CYCLE_END_UTC=$cycleEnd",
    "LAUNCHER_FAIL_COUNT=$launcherFailCount",
    "RESULTS_CSV=$resultsCsv",
    "LAUNCHER_LOG=$launcherLog"
) + $resultLines

$summary | Set-Content -LiteralPath $summaryTxt
Get-Content -LiteralPath $summaryTxt | Write-Host
