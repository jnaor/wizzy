#!/usr/bin/env python
import can
import os
import struct
import logging
from math import fabs

import rospy
from geometry_msgs.msg import Twist

INTERFACE = 'can0'
BITRATE = 105263

# empirically measured linear velocity in m/s
MAX_VELOCITY = 0.5

# empirically measured angular velocity in degrees/s
MAX_ANGULAR_VELOCITY = 40.0

# normalizaton constant (max value of reading)
MAX_VALUE = 100.0

ROS_TOPIC = 'cmd_vel'

cmd_pub = None

# report only if velocity greater than this (m/s) or large angular_velocity 
VELOCITY_THRESHOLD = 0.5

# report only if large velocity or angular velocity greater than this (deg/s)
ANGULAR_VELOCITY_THRESHOLD = 0.5


def sign(x):
    if x<=0:
        return -1
    return 1

def can_recv(iface):
    bus = can.interface.Bus(bustype='socketcan', channel=iface)
    y=0
    x=0 
	
    cmd_vel_msg = Twist()
    
    global cmd_pub	
    for msg in bus:
        
        if len(msg.data) == 6:
            data_array = struct.unpack('>6b', bytes(msg.data))
            y = data_array[4]
        elif len(msg.data) == 4:
            data_array = struct.unpack('>4b', bytes(msg.data))
            x = data_array[2]
        
        # normalize according to measured joystick characteristics
        xn, yn = min(abs(x), MAX_VALUE) * sign(x), min(abs(y), MAX_VALUE) * sign(y)

        # forward motion is positive
        velocity = (yn / MAX_VALUE) * MAX_VELOCITY

        # angular velocity. sign flip to make turn around left positive
        angular_velocity = MAX_ANGULAR_VELOCITY*(-xn / MAX_VALUE)

        logging.debug('vel: {}, ang: {}'.format(velocity, angular_velocity))

        cmd_vel_msg.linear.x = velocity
        cmd_vel_msg.angular.z = angular_velocity


        # publish if not zero
        if fabs(velocity) > VELOCITY_THRESHOLD or fabs(angular_velocity) > ANGULAR_VELOCITY_THRESHOLD:
            cmd_pub.publish(cmd_vel_msg)


# inactive; left here if someone forgets how to init on boot
def disable_iface(iface):
    os.system('sudo ip link set down {}'.format(iface))

# inactive; left here if someone forgets how to init on boot
def enable_iface(iface, bitrate):
    os.system('sudo /sbin/ip link set {} up type can bitrate {}'.format(iface, bitrate))



if __name__== "__main__":
    rospy.init_node('can_sniffer', log_level=rospy.DEBUG)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # set logging level
    logging.basicConfig(level=logging.DEBUG)
	
    # run
    can_recv(INTERFACE)
