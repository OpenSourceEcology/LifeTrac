$serial = "2E2C1209DABC240B"
$tmp = "/tmp/lifetrac_p0c"
$firmwareRoot = Join-Path $PSScriptRoot "LifeTrac-v25\DESIGN-CONTROLLER\firmware"
$helperDir = Join-Path $firmwareRoot "x8_lora_bootloader_helper"
$flasher = Join-Path $helperDir "stm32_an3155_flasher.py"
$image = Join-Path $firmwareRoot "murata_l072\build\firmware.bin"
adb -s $serial shell "mkdir -p $tmp"
adb -s $serial push "$helperDir\." $tmp/
adb -s $serial push "$flasher" $tmp/
adb -s $serial push "$image" $tmp/firmware.bin
adb -s $serial shell "echo fio | sudo -S bash $tmp/full_flash_pipeline.sh $tmp/firmware.bin $serial"