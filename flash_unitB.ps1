$serial = "2D0A1209DABC240B"
$tmp = "/tmp/lifetrac_p0c"
$firmwareRoot = Join-Path $PSScriptRoot "LifeTrac-v25\DESIGN-CONTROLLER\firmware"
$helperDir = Join-Path $firmwareRoot "x8_lora_bootloader_helper"
$flasher = Join-Path $PSScriptRoot "LifeTrac-v25\tools\stm32_an3155_flasher.py"
$image = Join-Path $firmwareRoot "murata_l072\build_ping_rx_active\lora_ping_rx_active.bin"

function Invoke-AdbStep {
	param(
		[Parameter(Mandatory = $true)]
		[string[]]$Arguments,
		[Parameter(Mandatory = $true)]
		[string]$StepName,
		[int]$Retries = 6
	)

	for ($attempt = 1; $attempt -le $Retries; $attempt++) {
		Write-Host "== $StepName (attempt $attempt/$Retries) =="
		& adb @Arguments
		if ($LASTEXITCODE -eq 0) {
			return
		}

		Write-Host "WARN: $StepName failed with exit code $LASTEXITCODE"
		if ($attempt -lt $Retries) {
			Start-Sleep -Seconds 2
			& adb -s $serial wait-for-device | Out-Null
			Start-Sleep -Seconds 1
		}
	}

	throw "ERROR: $StepName failed after $Retries attempts"
}

Invoke-AdbStep -StepName "create remote staging dir" -Arguments @("-s", $serial, "shell", "mkdir -p $tmp")
Invoke-AdbStep -StepName "push helper scripts" -Arguments @("-s", $serial, "push", "$helperDir\.", "$tmp/")
Invoke-AdbStep -StepName "push flasher" -Arguments @("-s", $serial, "push", $flasher, "$tmp/")
Invoke-AdbStep -StepName "push firmware image" -Arguments @("-s", $serial, "push", $image, "$tmp/lora_ping_rx_active.bin")
Invoke-AdbStep -StepName "run on-device flash pipeline" -Arguments @("-s", $serial, "shell", "echo fio | sudo -S bash $tmp/full_flash_pipeline.sh $tmp/lora_ping_rx_active.bin $serial")