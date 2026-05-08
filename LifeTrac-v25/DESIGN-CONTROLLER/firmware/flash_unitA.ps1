$serial = "2E2C1209DABC240B"
$tmp = "/tmp/lifetrac_p0c"
adb -s $serial shell "mkdir -p $tmp"
adb -s $serial push x8_lora_bootloader_helper/. $tmp/
adb -s $serial push ../../tools/stm32_an3155_flasher.py $tmp/
adb -s $serial push murata_l072/build_ping_tx/lora_ping_tx.bin $tmp/lora_ping_tx.bin
adb -s $serial shell "echo fio | sudo -S bash $tmp/full_flash_pipeline.sh $tmp/lora_ping_tx.bin $serial"