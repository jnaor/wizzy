#!/usr/bin/env python3

"""
Wizzy Button Client
===================
Based on file test_client.py
This program attempts to connect to all previously verified Flic buttons by this server.
Requires PYTHON 3 ! (flick library is py3)
"""

import time
import fliclib

import rospy
from std_msgs.msg import String

import logging
log = logging.getLogger(__name__)

def got_button(bd_addr):
    cc = fliclib.ButtonConnectionChannel(bd_addr)
    cc.on_button_single_or_double_click_or_hold = button_cb
    cc.on_connection_status_changed = on_connection_status_changed_cb
    client.add_connection_channel(cc)

def init_buttons(items):
    log.debug("*** INIT_BUTTONS")  
    log.debug(items)
    for bd_addr in items["bd_addr_of_verified_buttons"]:
        got_button(bd_addr)

def button_cb (channel, click_type, was_queued, time_diff) : 
    log.debug(channel.bd_addr + " " + str(click_type))
    if click_type.name == "ButtonSingleClick" :
        print ("*** Once !")
    elif click_type.name == "ButtonDoubleClick" :
        print ("*** Twice !")
    elif click_type.name == "ButtonHold" :
        print ("*** Hold !")
    pub.publish(click_type.name)
        
def on_connection_status_changed_cb(channel, connection_status, disconnect_reason) :
    print(channel.bd_addr + " " + str(connection_status) + \
          (" " + str(disconnect_reason) if connection_status == fliclib.ConnectionStatus.Disconnected else ""))
    
    
if __name__ == "__main__" :
   log.error ("*** Init flick client")
   
    # Initialize publisher 
   rospy.init_node('flic_button', log_level=rospy.DEBUG)
   pub = rospy.Publisher('/wizzy/flic_btn', String, queue_size=10)   
   
   # initialize flic client to None
   client = None
   while client is None:
       try: 
           client = fliclib.FlicClient("localhost")
           rospy.loginfo('FLIC connection established')
       except ConnectionRefusedError:
           rospy.logwarn("can't find FLIC server; will keep trying")
           client = None
           time.sleep(3)

   # Initialize buttons with the pre-paired buttons in the bin directory, wizzy_buttons.db file
   client.get_info(init_buttons)
   client.handle_events()
