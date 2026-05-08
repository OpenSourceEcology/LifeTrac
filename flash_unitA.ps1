$serial = "2E2C1209DABC240B"
$tmp = "/tmp/lifetrac_p0c"
$firmwareRoot = Join-Path $PSScriptRoot "LifeTrac-v25\DESIGN-CONTROLLER\firmware"
$helperDir = Join-Path $firmwareRoot "x8_lora_bootloader_helper"
$flasher = Join-Path $PSScriptRoot "LifeTrac-v25\tools\stm32_an3155_flasher.py"
$image = Join-Path $firmwareRoot "murata_l072\build_ping_tx_active\lora_ping_tx_active.bin"
adb -s $serial shell "mkdir -p $tmp"
adb -s $serial push "$helperDir\." $tmp/
adb -s $serial push "$flasher" $tmp/
adb -s $serial push "$image" $tmp/lora_ping_tx_active.bin
adb -s $serial shell "echo fio | sudo -S bash $tmp/full_flash_pipeline.sh $tmp/lora_ping_tx_active.bin $serial"