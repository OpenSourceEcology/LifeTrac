# Tail /dev/kmsg (root) into an fsync'd file; survives a PMIC power-cycle.
import os, sys, time
out = open(sys.argv[1], "a")
fd = os.open("/dev/kmsg", os.O_RDONLY | os.O_NONBLOCK)
# skip the backlog
while True:
    try:
        os.read(fd, 8192)
    except BlockingIOError:
        break
out.write("%.3f KMSG-TAIL-START\n" % time.monotonic()); out.flush(); os.fsync(out.fileno())
while True:
    try:
        b = os.read(fd, 8192)
        out.write("%.3f %s" % (time.monotonic(), b.decode("utf-8", "replace")))
        out.flush(); os.fsync(out.fileno())
    except BlockingIOError:
        time.sleep(0.2)
