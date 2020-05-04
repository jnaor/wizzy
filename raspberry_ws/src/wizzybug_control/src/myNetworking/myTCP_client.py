#!/usr/bin/env python
 
import socket
import time

TCP_IP = '192.168.1.100'
TCP_PORT = 5000
BUFFER_SIZE = 1024
MESSAGE = "Hello, World!"

counter = 0

# Init
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

# Periodic message send
while counter < 5 :
    s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    print ("received data: " + data)
    time.sleep(1)
    counter += 1

# Close socket - Unreachable code
s.close()

