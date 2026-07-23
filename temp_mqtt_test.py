import socket
s = socket.socket()
s.settimeout(2.0)
res = s.connect_ex(('192.168.1.79', 1883))
print("CONNECT_RESULT=", res)
