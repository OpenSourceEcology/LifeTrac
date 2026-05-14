$file = 'c:\Users\dorkm\AppData\Roaming\Code\User\workspaceStorage\194d4d9a91b922298668981e53366abf\GitHub.copilot-chat\transcripts\fc0fbbcf-1193-4af9-9fdb-ac6ccb1241d1.jsonl'
$lines = Get-Content $file
foreach ($line in $lines) {
    if ($line -match "recover_l072_opt.sh|wunprot") {
        $obj = ConvertFrom-Json $line
        if ($obj.type -eq "tool.execution_start" -and $obj.data.toolName -eq "run_in_terminal") {
            Write-Host "$($obj.timestamp) | $($obj.data.arguments.command)"
        }
    }
}
