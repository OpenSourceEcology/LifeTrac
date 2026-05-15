$ErrorActionPreference = "Stop"

Write-Host "Starting Image Capture Test on Portenta X8"

# 1. Verify camera device exists
Write-Host "Checking for /dev/lifetrac-c2..."
$devCheck = adb shell "sudo ls /dev/lifetrac-c2 2>/dev/null"
if ([string]::IsNullOrWhiteSpace($devCheck) -or $devCheck -notmatch "lifetrac-c2") {
    Write-Warning "Camera /dev/lifetrac-c2 not found on device! Please ensure the camera is plugged in and udev rules are loaded."
    exit 1
}
Write-Host "Camera device found."

# 2. Run a lightweight Alpine container, install FFmpeg, and grab 1 frame
Write-Host "Spawning Docker container to capture MJPEG frame (this may take a moment to pull Alpine on first run)..."
$dockerCmd = "sudo docker run --rm --device=/dev/lifetrac-c2 -v /var/rootdirs/home/fio:/host_home alpine sh -c 'apk add --no-cache ffmpeg && ffmpeg -y -f v4l2 -input_format mjpeg -i /dev/lifetrac-c2 -vframes 1 /host_home/test_frame.jpg'"

# Note: We use adb shell without -t here to capture output properly in Powershell
adb shell $dockerCmd

# 3. Pull the captured image back to the host machine
Write-Host "Pulling test_frame.jpg to local workspace..."
# Clean up any previous local file
if (Test-Path "test_frame.jpg") { Remove-Item "test_frame.jpg" -Force }

adb pull /var/rootdirs/home/fio/test_frame.jpg .\test_frame.jpg

if (Test-Path "test_frame.jpg") {
    Write-Host "Success! Image captured and saved to $(Resolve-Path .\test_frame.jpg)"
} else {
    Write-Warning "Failed to retrieve the captured image."
}
