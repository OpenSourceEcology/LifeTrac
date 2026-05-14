$roots = @(
    $env:ProgramFiles,
    ${env:ProgramFiles(x86)},
    $env:LOCALAPPDATA,
    'C:\ST',
    'C:\STM32CubeProgrammer'
) | Where-Object { $_ -and (Test-Path -LiteralPath $_) }

$matches = foreach ($root in $roots) {
    Get-ChildItem -LiteralPath $root -Filter STM32_Programmer_CLI.exe -Recurse -ErrorAction SilentlyContinue
}

if (-not $matches) {
    Write-Host 'STM32_Programmer_CLI.exe not found in standard Program Files / LocalAppData / C:\ST locations.'
    exit 0
}

$matches | Select-Object -ExpandProperty FullName