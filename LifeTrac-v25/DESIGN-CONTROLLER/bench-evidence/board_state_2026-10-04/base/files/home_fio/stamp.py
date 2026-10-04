# stdin -> stdout + fsync'd file, each line prefixed with monotonic seconds.
import os, sys, time
out = open(sys.argv[1], "a")
for line in sys.stdin:
    s = "%.3f %s" % (time.monotonic(), line)
    sys.stdout.write(s); sys.stdout.flush()
    out.write(s); out.flush(); os.fsync(out.fileno())
