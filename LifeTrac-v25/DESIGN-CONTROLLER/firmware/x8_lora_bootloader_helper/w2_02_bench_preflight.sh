#!/bin/sh
# W2-02 bench preflight: report state of camera, ffmpeg, python deps,
# helper toolkit, and USB devices on a Portenta X8.  Pure read-only.
echo "=== W2-02 BENCH PREFLIGHT ==="
echo "--- uname ---"
uname -a
echo "--- /dev/video* ---"
ls -l /dev/video* 2>/dev/null || echo "(no video nodes)"
echo "--- video4linux names ---"
for n in /sys/class/video4linux/video*/name; do
  [ -e "$n" ] && echo "$n: $(cat "$n")"
done
echo "--- lsusb ---"
which lsusb >/dev/null 2>&1 && lsusb || echo "(no lsusb)"
echo "--- Kurokesu sysfs ---"
for d in /sys/bus/usb/devices/*; do
  v=$(cat "$d/idVendor" 2>/dev/null)
  p=$(cat "$d/idProduct" 2>/dev/null)
  if [ "$v" = "16d0" ] && [ "$p" = "0ed4" ]; then
    echo "FOUND Kurokesu C2 at $d"
  fi
done
echo "--- /tmp helpers ---"
ls -l /tmp/ffmpeg /tmp/lifetrac_p0c 2>/dev/null || true
echo "--- python3 ---"
python3 -V 2>&1
python3 - <<'PY' 2>&1
mods = ["PIL", "PIL.Image", "numpy", "struct", "ctypes"]
for m in mods:
    try:
        x = __import__(m)
        v = getattr(x, "__version__", "(builtin)")
        print(f"  {m:20s} OK  {v}")
    except Exception as e:
        print(f"  {m:20s} MISSING ({e.__class__.__name__})")
PY
echo "--- /dev/ttymxc3 ---"
ls -l /dev/ttymxc3 2>/dev/null || echo "(no ttymxc3)"
echo "--- L072 firmware presence ---"
ls -l /tmp/lifetrac_p0c/firmware.bin /tmp/lifetrac_p0c/method_h_stage2_tx_probe.py 2>/dev/null || \
  echo "(L072 helper toolkit not pushed)"
echo "=== DONE ==="
