#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
import socket
import errno
 
class backend  :
    # Netwroking backend constants
    TCP_IP      = '192.168.1.100'
    TCP_PORT    = 5000
    BUFFER_SIZE = 20  # Normally 1024, but we want fast response

    def __init__(Self) :
        self.mySock = null
        
        # Subscribe 
        rospy.Subscriber("/test", String, self.update_data_cb)

    def update_data_cb(self, msg) :
        """
        Data received from decision making node (DM), that will be 
        sent to the Frontend using the TCP server connection`
        """
        
    def init_network(self) :
        self.mySock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mySock.bind((TCP_IP, TCP_PORT))

        print ("==> Listening on " + TCP_IP + " / " + TCP_PORT)
        self.mySock.listen(1)
        
    def server_loop(self) :
        myCounter  = 1

        conn, addr = self.mySock.accept()
        print ('==> Received a connection from address: ' + str(addr))                
        
        while 1:
            data = conn.recv(BUFFER_SIZE)
            if data: 
                print ("## received data:" +  str(data))
            conn.send(str(myCounter) + "\n")  # echo
            myCounter += 1    

        # Non reachable code. Need to take care of case where remote client socket is closed,
        # and to close connection        
        conn.close()
        
if __name__ == '__main__' :
    backendObj = backend()
    backendObj.init_network()
    backend.server_loop()
    