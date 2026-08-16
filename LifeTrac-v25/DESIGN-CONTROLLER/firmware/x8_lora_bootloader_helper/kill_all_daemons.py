import os
import psutil

print("Scanning processes...")
for p in psutil.process_iter(['pid', 'name', 'cmdline']):
    try:
        cmd = " ".join(p.info['cmdline'] or [])
        if any(x in cmd for x in ['image_tx', 'image_rx', 'publish_synthetic', 'run_concurrent', 'run_live_radio']):
            # Print AFTER the kill so the output is trustworthy for bench
            # triage — a permission error or pid race must not read as a
            # successful kill.
            try:
                os.kill(p.info['pid'], 9)
            except Exception as exc:
                print(f"FAILED to kill PID {p.info['pid']}: {exc}: {cmd}")
            else:
                print(f"Killed PID {p.info['pid']}: {cmd}")
    except Exception:
        pass
print("Scan done.")
