import socket

HOST = "127.0.0.1"
PORT = 6666
MESSAGE = b"ding_dong"

print("UDP target IP: %s" % HOST)
print("UDP target port: %s" % PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
sock.sendall(b'ding_dong')