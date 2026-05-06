# Software Observability Plan: Python Msgpack-RPC Client

**Date:** 2026-05-06
**Status:** Proposed Plan
**Objective:** Establish a software-only observation path for the M4 (Arduino) to Linux bridge on the Portenta X8 by interfacing with `m4_proxy` via msgpack-rpc.

## 1. Background

The `tractor_h7` firmware successfully loops `Serial2` (LoRa) to `SerialRPC`. On the Linux side, `m4_proxy` receives these OpenAMP/RPMsg buffers and wraps them in a MessagePack RPC (msgpack-rpc) TCP server on ports 5000 and 5001. To read the LoRa bytes on Linux without hardware tools, we must write a Python client to negotiate with `m4_proxy` and decode the payload.

## 2. Prerequisites

- Portenta X8 connected via ADB (`adb shell`).
- Internet access on the Portenta X8 (for `pip install` if not already offline-available).
- Python 3 installed on the Portenta X8 rootfs.

## 3. Implementation Steps

### Step 1: Verify Environment & Install Dependencies
Access the Portenta X8 network and ensure Python dependencies can be met.
```bash
adb -s 2D0A1209DABC240B shell
# Verify Python is available
python3 --version
# Install the MessagePack RPC client library via pip
sudo pip3 install mprpc msgpack
```

### Step 2: Write the Python Client Script (`lora_sniff.py`)
Create a script on the Linux filesystem that connects to the local `m4_proxy` service. 

```python
import msgpackrpc
import time

# Connect to the m4_proxy msgpack-rpc server
# Typically port 5000 or 5001 is for the service on Portenta proxy
client = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 5001))

print("Listening to m4_proxy for LoRa traffic...")

try:
    # m4_proxy exposes RPC methods to interact with the Arduino core.
    # The M4 sends RPC.send("tty", buffer).
    # We will poll the proxy or setup an async server listener if the proxy pushes callbacks.
    while True:
         # Depending on the Exact proxy API, we invoke a read 
         # e.g., res = client.call('tty_read')
         # if res: print(res.decode('utf-8'))
         time.sleep(0.1)
         
except KeyboardInterrupt:
    print("Stopping client.")
client.close()
```
*Note: The exact RPC method name exposed by `m4_proxy` will be verified during execution.*

### Step 3: Run the Script and Validate with Heartbeat
1. Flush the M4 core with the 1Hz Heartbeat beacon.
2. Push the script via ADB: `adb push lora_sniff.py /home/fio/lora_sniff.py`
3. Run the script: `adb shell "python3 /home/fio/lora_sniff.py"`

If successful, the terminal will print the heartbeat strings every second, proving a complete end-to-end software observability channel from the LoRa traces all the way to the Linux terminal.

## 4. Alternative (Go Client)
Because `m4_proxy` is built in Go (`github.com/msgpack-rpc/msgpack-rpc-go`), if Python dependencies fail to install via `pip` on the Alpine Linux image, we can compile a static Go binary on the host machine and simply push the self-contained executable to the X8.