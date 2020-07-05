#!/usr/bin/env python
"""
From :
https://github.com/numato/samplecode/blob/master/RelayAndGPIOModules/USBRelayAndGPIOModules/python/usbrelay1_2_4_8/relaywrite.py
Input : Expects to receive command message with the strings "on" or "off", 
        or from the Flic button ("ButtonSingleClick" for ON and "ButtonDoubleClick" for OFF.
"""
import rospy
from std_msgs.msg import String
   
import sys
import serial

PORT_NAME = "/dev/ttyACM0"
RELAYNUM  = "0"                   # Support only one relay 

def sendCmdToRelay(cmdStr) :
   #Open port for communication
    serPort = serial.Serial(PORT_NAME, 19200, timeout=1)

    #Send the command
    # serPort.write("relay "+ str(relayCmd) +" "+ str(relayNum) + "\n\r")
    str_cmd = "relay "+ cmdStr +" "+ RELAYNUM + "\n\r"
    serPort.write(str.encode(str_cmd))

    print ("Command sent...")

    #Close the port
    serPort.close()    

def callback(msg):
    """
    Message received from the DM. The msg.data matches the string
    expected by the USB Relay driver (either "on" or "off" string,
    so no need to translate it.
    """
    print ("USB Relay - I heard " + msg.data)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
    sendCmdToRelay(msg.data)
     

#def callback_Button(msg):
#    """
#    Message received from the user via the Flic button
#    """
#    if msg.data == "ButtonSingleClick" :
#        print ("on")
#        sendCmdToRelay("on")
#    elif msg.data == "ButtonDoubleClick" :
#        sendCmdToRelay("off")        
#        print ("off")    

def listener():
    rospy.init_node('usb_relay_listener', anonymous=True)
    
    # DM command topic
    rospy.Subscriber("usb_relay_command", String, callback)
    
    # Flic button command topic
#    rospy.Subscriber("/wizzy/flic_btn",   String, callback_Button)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
    
if __name__ == "__main__" :
    print ("Starting USB relay ")
    rospy.loginfo(rospy.get_caller_id() + "usb relay listener starting")
    listener()
    
