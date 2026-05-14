$file = 'c:\Users\dorkm\AppData\Roaming\Code\User\workspaceStorage\194d4d9a91b922298668981e53366abf\GitHub.copilot-chat\transcripts\fc0fbbcf-1193-4af9-9fdb-ac6ccb1241d1.jsonl'
$lines = Get-Content $file
foreach ($line in $lines) {
    if ($line -match "wunprot") {
        $obj = ConvertFrom-Json $line
        Write-Host "$($obj.timestamp) | $($obj.role)"
        Write-Host ($obj.message).Substring(0, [math]::Min(100, $obj.message.Length))
    }
}
