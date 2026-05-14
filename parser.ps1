 = Get-Content 'c:\Users\dorkm\AppData\Roaming\Code\User\workspaceStorage\194d4d9a91b922298668981e53366abf\GitHub.copilot-chat\transcripts\fc0fbbcf-1193-4af9-9fdb-ac6ccb1241d1.jsonl'  
foreach ( in ) { if ( -match 'wunprot') {  = ConvertFrom-Json ; Write-Host .timestamp '-' .role; Write-Host (.message).Substring(0, [math]::Min(100, .message.Length)) } }  
