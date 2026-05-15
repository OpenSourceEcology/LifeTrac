import sys
try:
    import fcntl
    import mmap
    print("Core modules found.")
except Exception as e:
    print("Error:", e)
    
try:
    import cv2
    print("cv2 found!")
except:
    pass

try:
    from v4l2 import *
    print("v4l2 found!")
except:
    pass
