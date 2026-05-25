import sys
print("sys.path:", sys.path[:3])
try:
    import image_tx_daemon
    print("IMPORT image_tx_daemon OK")
except Exception as e:
    import traceback
    traceback.print_exc()
    sys.exit(1)
