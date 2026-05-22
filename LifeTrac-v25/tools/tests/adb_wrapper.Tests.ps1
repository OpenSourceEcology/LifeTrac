#requires -Version 5.1
<#
.SYNOPSIS
    Pester regression fixture for Invoke-AdbScoped (P2).

.DESCRIPTION
    The scoped-EAP adb wrapper in
    LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w2_02_image_over_lora_end_to_end_v2.ps1
    keeps adb's noisy success-on-stderr from aborting the orchestrator
    while still aborting on genuine adb failures. Per v4.0 §3 of the
    2026-05-21 Open Problems doc, the wrapper must remain regression-
    locked: a refactor that silently turns "rc != 0" into "ignored"
    re-opens the same trap P2 was created to close.

    This fixture is intentionally side-effect-free where possible:
      - the bad-serial case never reaches a real device;
      - the smoke-success case is auto-skipped unless
        $env:LIFETRAC_TX_SERIAL is set, so CI without bench hardware
        still passes.

    Run locally:
        Invoke-Pester -Path LifeTrac-v25/tools/tests/adb_wrapper.Tests.ps1
#>

BeforeAll {
    $script:OrchPath = Join-Path $PSScriptRoot '..\..\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\run_w2_02_image_over_lora_end_to_end_v2.ps1'
    if (-not (Test-Path -LiteralPath $script:OrchPath)) {
        throw "Orchestrator not found at $script:OrchPath"
    }
    # Dot-source the function out of the orchestrator without executing its
    # main body: extract the Invoke-AdbScoped function text via the AST.
    $errs = $null
    $tokens = $null
    $ast = [System.Management.Automation.Language.Parser]::ParseFile(
        $script:OrchPath, [ref]$tokens, [ref]$errs)
    if ($errs.Count -gt 0) {
        throw "Orchestrator failed to parse: $($errs | ForEach-Object { $_.Message } | Out-String)"
    }
    $fnAst = $ast.Find({
        param($n)
        $n -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
        $n.Name -eq 'Invoke-AdbScoped'
    }, $true)
    if (-not $fnAst) {
        throw "Invoke-AdbScoped function not found in orchestrator (P2 wrapper missing?)."
    }
    Invoke-Expression $fnAst.Extent.Text
}

Describe 'Invoke-AdbScoped (P2 wrapper)' {

    It 'throws when given a non-existent serial (genuine adb failure still fatal)' {
        # adb returns rc!=0 for unknown serial; the wrapper must surface this.
        { Invoke-AdbScoped -Serial 'P2_NONEXISTENT_SERIAL_ABCD0123' `
            -AdbArgs @('shell', 'echo', 'ok') } |
            Should -Throw -ExpectedMessage 'adb*failed rc=*'
    }

    It 'returns the offending rc without throwing when -NoThrow is set' {
        $r = Invoke-AdbScoped -Serial 'P2_NONEXISTENT_SERIAL_ABCD0123' `
            -AdbArgs @('shell', 'echo', 'ok') -NoThrow
        $r.Rc | Should -Not -Be 0
    }

    It 'returns a pscustomobject with Rc and Output properties' {
        # `adb version` always succeeds and writes to stdout, no serial needed.
        $r = Invoke-AdbScoped -AdbArgs @('version')
        $r | Should -BeOfType ([pscustomobject])
        $r.PSObject.Properties.Name | Should -Contain 'Rc'
        $r.PSObject.Properties.Name | Should -Contain 'Output'
        $r.Rc | Should -Be 0
    }

    It 'restores $ErrorActionPreference after the call' {
        $before = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Stop'
            [void](Invoke-AdbScoped -AdbArgs @('version'))
            $ErrorActionPreference | Should -Be 'Stop'
        } finally {
            $ErrorActionPreference = $before
        }
    }

    It 'survives a successful push that writes its summary to stderr' -Skip:(-not $env:LIFETRAC_TX_SERIAL) {
        # Real hardware required: a push of a tiny dummy file should NOT
        # throw even though adb prints "1 file pushed, 0 skipped. X MB/s …"
        # to stderr. This is the exact failure mode P2 closes.
        $tmp = New-TemporaryFile
        try {
            'p2-wrapper-smoke' | Set-Content -LiteralPath $tmp
            { Invoke-AdbScoped -Serial $env:LIFETRAC_TX_SERIAL `
                -AdbArgs @('push', $tmp.FullName, '/tmp/p2_wrapper_smoke.txt') } |
                Should -Not -Throw
        } finally {
            Remove-Item -LiteralPath $tmp -Force -ErrorAction SilentlyContinue
        }
    }

    It 'always merges stderr into the Output stream so callers using `*>&1 | Tee-Object` see no NativeCommandError block' -Skip:(-not $env:LIFETRAC_TX_SERIAL) {
        # 2026-05-22 P2 follow-up: the original wrapper only merged stderr
        # when -MergeStderr was passed. Even with EAP=Continue, a native
        # command writing to stderr still emits an ErrorRecord that
        # `*>&1 | Tee-Object` renders as a red "NativeCommandError" block
        # in the bench log — the very artifact P2 set out to eliminate.
        # The fix is to ALWAYS merge stderr: `& adb @cmd 2>&1`. This test
        # asserts the wrapper never leaks an ErrorRecord on a successful
        # push by capturing the error stream redirected to output and
        # confirming no [System.Management.Automation.ErrorRecord] objects
        # were produced.
        $tmp = New-TemporaryFile
        try {
            'p2-merge-stderr-smoke' | Set-Content -LiteralPath $tmp
            $captured = & {
                Invoke-AdbScoped -Serial $env:LIFETRAC_TX_SERIAL `
                    -AdbArgs @('push', $tmp.FullName, '/tmp/p2_merge_stderr_smoke.txt')
            } 2>&1
            $errs = $captured | Where-Object {
                $_ -is [System.Management.Automation.ErrorRecord]
            }
            $errs.Count | Should -Be 0
        } finally {
            Remove-Item -LiteralPath $tmp -Force -ErrorAction SilentlyContinue
        }
    }
}
