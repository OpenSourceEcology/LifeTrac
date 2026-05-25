param(
    [string]$AdbSerial = "2E2C1209DABC240B",
    [string]$Dev = "/dev/video1"
)
$ErrorActionPreference = "Stop"
$pw = "fio"
$cmd = @"
echo $pw | sudo -S -p '' docker run --rm --device=$Dev -v /tmp/lifetrac_strict:/work -w /work --entrypoint /work/ffmpeg hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 -f v4l2 -list_formats all -i $Dev 2>&1 | tail -40
echo '--- capture frame ---'
echo $pw | sudo -S -p '' docker run --rm --device=$Dev -v /tmp/lifetrac_strict:/work -w /work --entrypoint /work/ffmpeg hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 -y -f v4l2 -i $Dev -frames:v 1 /work/probe_frame.jpg 2>&1 | tail -20
echo '--- file size ---'
ls -la /tmp/lifetrac_strict/probe_frame.jpg 2>&1
"@
adb -s $AdbSerial shell $cmd
adb -s $AdbSerial pull /tmp/lifetrac_strict/probe_frame.jpg .\probe_frame.jpg 2>&1 | Out-Host
Write-Host "Saved probe_frame.jpg"
