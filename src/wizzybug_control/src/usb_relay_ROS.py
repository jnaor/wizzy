#!/usr/bin/env python
"""
From :
https://github.com/numato/samplecode/blob/master/RelayAndGPIOModules/USBRelayAndGPIOModules/python/usbrelay1_2_4_8/relaywrite.py
Input : Expects to receive command message with the strings "on" or "off"
"""


import rospy
from std_msgs.msg import String
   
import sys
import serial

PORT_NAME = "/dev/ttyACM0"
RELAYNUM  = "0"                   # Support only one relay 

def callback(data):
    print ("USB Relay - I heard " + data.data)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
    #Open port for communication
    serPort = serial.Serial(PORT_NAME, 19200, timeout=1)

    #Send the command
    # serPort.write("relay "+ str(relayCmd) +" "+ str(relayNum) + "\n\r")
    str_cmd = "relay "+ data.data +" "+ RELAYNUM + "\n\r"
    serPort.write(str.encode(str_cmd))

    print ("Command sent...")

    #Close the port
    serPort.close()    
       
def listener():
    rospy.init_node('usb_relay_listener', anonymous=True)
    rospy.Subscriber("usb_relay_command", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
    
if __name__ == "__main__" :
    print ("Starting USB relay ")
    rospy.loginfo(rospy.get_caller_id() + "usb relay listener starting")
    listener()
    