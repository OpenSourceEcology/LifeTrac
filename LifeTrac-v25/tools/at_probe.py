import serial
import time
import sys

# Fast try to export gpio163
try:
    with open('/sys/class/gpio/export', 'w') as f:
        f.write('163')
except Exception:
    pass

try:
    with open('/sys/class/gpio/gpio163/direction', 'w') as f:
        f.write('out')
except Exception:
    pass

def reset_modem():
    print("Resetting modem...")
    try:
        with open('/sys/class/gpio/gpio163/value', 'w') as f:
            f.write('0')
        time.sleep(0.1)
        with open('/sys/class/gpio/gpio163/value', 'w') as f:
            f.write('1')
        time.sleep(0.5)
    except Exception as e:
        print("Failed to reset:", e)

reset_modem()
print("Opening serial...")
s = serial.Serial('/dev/ttymxc3', 19200, timeout=1)

def send_cmd(cmd):
    print(f"--> {cmd}")
    s.write((cmd + '\r\n').encode())
    time.sleep(0.2)
    resp = s.read_all().decode(errors='ignore')
    print(f"<-- {resp.strip()}")

send_cmd('AT')
send_cmd('AT+VER?')
send_cmd('AT+DEV?')
send_cmd('AT+APPEUI')
sys.exit(0)
