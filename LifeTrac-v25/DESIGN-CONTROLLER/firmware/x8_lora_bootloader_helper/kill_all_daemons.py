import os
import psutil

print("Scanning processes...")
for p in psutil.process_iter(['pid', 'name', 'cmdline']):
    try:
        cmd = " ".join(p.info['cmdline'] or [])
        if any(x in cmd for x in ['image_tx', 'image_rx', 'publish_synthetic', 'run_concurrent', 'run_live_radio']):
            print(f"Killing PID {p.info['pid']}: {cmd}")
            os.kill(p.info['pid'], 9)
    except Exception as exc:
        pass
print("Scan done.")
